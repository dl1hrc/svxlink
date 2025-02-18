/**
@file	 LocationInfo.cpp
@brief   Contains the infrastructure for APRS based EchoLink status updates
@author  Adi/DL1HRC and Steve/DH1DM
@date	 2009-03-12

\verbatim
SvxLink - A Multi Purpose Voice Services System for Ham Radio Use
Copyright (C) 2003-2025 Tobias Blomberg / SM0SVX

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
\endverbatim
*/



/****************************************************************************
 *
 * System Includes
 *
 ****************************************************************************/

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <limits>
#include <string>
#include <sys/time.h>
#include <regex.h>
#include <iomanip>

/****************************************************************************
 *
 * Project Includes
 *
 ****************************************************************************/

#include <AsyncConfig.h>
#include <AsyncTimer.h>


/****************************************************************************
 *
 * Local Includes
 *
 ****************************************************************************/

#include "version/SVXLINK.h"
#include "LocationInfo.h"
#include "AprsTcpClient.h"
#include "AprsUdpClient.h"
#include "GpsdTcpClient.h"


/****************************************************************************
 *
 * Namespaces to use
 *
 ****************************************************************************/

using namespace std;
using namespace Async;
using namespace EchoLink;


  // Put all locally defined types, functions and variables in an anonymous
  // namespace to make them file local. The "static" keyword has been
  // deprecated in C++ so it should not be used.
namespace
{

/****************************************************************************
 *
 * Defines & typedefs
 *
 ****************************************************************************/

 #define NMEA_VERSION "18022025"

/****************************************************************************
 *
 * Prototypes for local functions
 *
 ****************************************************************************/

void print_error(const string &name, const string &variable,
                 const string &value, const string &example = "");


/****************************************************************************
 *
 * Local Global Variables
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Local class definitions
 *
 ****************************************************************************/


} // End of anonymous namespace

/****************************************************************************
 *
 * Exported Global Variables
 *
 ****************************************************************************/

# define M_PI           3.14159265358979323846  /* pi */
# define RADIUS         6378.16

/****************************************************************************
 *
 * Public member functions
 *
 ****************************************************************************/

LocationInfo* LocationInfo::_instance = nullptr;


LocationInfo::LocationInfo(void)
{
  aprs_stats_timer.expired.connect(sigc::hide(
      mem_fun(*this, &LocationInfo::sendAprsStatistics)));
} /* LocationInfo::LocationInfo */


bool LocationInfo::initialize(Async::Config& cfg, const std::string& cfg_name)
{
    // check if already initialized
  if (LocationInfo::_instance->has_instance()) return false;

  bool init_ok = true;

  LocationInfo::_instance = new LocationInfo();
  auto& loc_cfg = LocationInfo::_instance->loc_cfg;

  std::string callsign;
  callsign = cfg.getValue(cfg_name, "CALLSIGN");

  if (callsign.find("EL-") != string::npos)
  {
    loc_cfg.prefix = "L";
  }
  else if (callsign.find("ER-") != string::npos)
  {
    loc_cfg.prefix = "R";
  }
  else
  {
    cerr << "*** ERROR: variable CALLSIGN must have a prefix (ER- or EL-)" <<
         " to indicate that is an Echolink station.\n" <<
         "Example: CALLSIGN=ER-DL1ABC\n";
    return false;
  }
  if (callsign.erase(0,3).length() < 4)
  {
    cerr << "*** ERROR: variable CALLSIGN in section " <<
        cfg_name << " is missing or wrong\nExample: CALLSIGN=ER-DL1ABC\n";
    return false;
  }
  loc_cfg.mycall = callsign;

  cfg.getValue(cfg_name, "COMMENT", loc_cfg.comment);
  if (loc_cfg.comment.size() > 36)
  {
    std::cerr << "*** WARNING: The APRS comment specified in " << cfg_name
              << "/COMMENT is too long. The maximum length is 36 characters."
              << std::endl;
    loc_cfg.comment.erase(36);
  }

  loc_cfg.debug = false;
  cfg.subscribeValue(cfg_name, "DEBUG",
      loc_cfg.debug,
      [](bool debug) {
        LocationInfo::_instance->loc_cfg.debug = debug;
      });  

  unsigned dest_num = 1;
  //cfg.getValue(cfg_name, "DESTINATION_NUM", 1U, 9U, dest_num);
  std::stringstream dest_ss;
  dest_ss << "APSVX" << dest_num;
  loc_cfg.destination = dest_ss.str();

  init_ok &= LocationInfo::_instance->parsePosition(cfg, cfg_name);
  init_ok &= LocationInfo::_instance->parseStationHW(cfg, cfg_name);
  init_ok &= LocationInfo::_instance->parsePath(cfg, cfg_name);
  init_ok &= LocationInfo::_instance->parseClients(cfg, cfg_name);

  //cfg.getValue(cfg_name, "DESTINATION_NUM", 1U, 9U, dest_num);
  LocationInfo::_instance->loc_cfg.destination = dest_ss.str();

  std::string value;
  if(cfg.getValue(cfg_name, "NMEA_DEVICE", value))
  {
    init_ok &=  LocationInfo::_instance->initNmeaDev(cfg, cfg_name);
  }
  else if (cfg.getValue(cfg_name, "GPSD", value))
  {
    init_ok &= LocationInfo::_instance->initGpsdClient(cfg, cfg_name);
  }

  auto& siv = LocationInfo::_instance->sinterval;
  cfg.getValue(cfg_name, "STATISTICS_INTERVAL", 5U, 60U, siv);
  LocationInfo::_instance->startStatisticsTimer(siv * 60 * 1000);

  cfg.getValue(cfg_name, "STATISTICS_LOGIC", LocationInfo::_instance->slogic);

  cfg.getValue(cfg_name, "FILTER", loc_cfg.filter);

  cfg.getValue(cfg_name, "SYMBOL", loc_cfg.symbol);
  if (!loc_cfg.symbol.empty() && (loc_cfg.symbol.size() != 2))
  {
    std::cerr << "*** ERROR: The APRS symbol specified in " << cfg_name
              << "/SYMBOL must be exactly two characters or empty"
              << std::endl;
    return false;
  }

  LocationInfo::_instance->initExtPty(cfg.getValue(cfg_name, "PTY_PATH"));
  
  if( !init_ok )
  {
    delete LocationInfo::_instance;
    LocationInfo::_instance = NULL;
  }

  return init_ok;

} /* LocationInfo::initialize */


void LocationInfo::updateDirectoryStatus(StationData::Status status)
{
  ClientList::const_iterator it;
  for (it = clients.begin(); it != clients.end(); it++)
  {
    (*it)->updateDirectoryStatus(status);
  }
} /* LocationInfo::updateDirectoryStatus */


void LocationInfo::updateQsoStatus(int action, const string& call,
                                   const string& info, list<string>& call_list)
{
  ClientList::const_iterator it;
  for (it = clients.begin(); it != clients.end(); it++)
  {
    (*it)->updateQsoStatus(action, call, info, call_list);
  }
} /* LocationInfo::updateQsoStatus */


void LocationInfo::update3rdState(const string& call, const string& info)
{
  ClientList::const_iterator it;
  for (it = clients.begin(); it != clients.end(); it++)
  {
    (*it)->update3rdState(call, info);
  }
} /* LocationInfo::update3rdState */


void LocationInfo::igateMessage(const std::string& info)
{
  ClientList::const_iterator it;
  for (it = clients.begin(); it != clients.end(); it++)
  {
    (*it)->igateMessage(info);
  }
} /* LocationInfo::igateMessage */


string LocationInfo::getCallsign(void)
{
  return loc_cfg.mycall;
} /* LocationInfo::getCallsign */


bool LocationInfo::getTransmitting(const std::string &name)
{
   return aprs_stats[name].tx_on;
} /* LocationInfo::getTransmitting */


void LocationInfo::setTransmitting(const std::string &name, struct timeval tv,
                                     bool state)
{
   aprs_stats[name].tx_on = state;
   if (state)
   {
      aprs_stats[name].tx_on_nr++;
      aprs_stats[name].last_tx_sec = tv;
   }
   else
   {
      aprs_stats[name].tx_sec += ((tv.tv_sec -
                aprs_stats[name].last_tx_sec.tv_sec) +
            (tv.tv_usec - aprs_stats[name].last_tx_sec.tv_usec)/1000000.0);
   }
} /* LocationInfo::setTransmitting */


void LocationInfo::setReceiving(const std::string &name, struct timeval tv,
                                 bool state)
{
   aprs_stats[name].squelch_on = state;
   if (state)
   {
      aprs_stats[name].rx_on_nr++;
      aprs_stats[name].last_rx_sec = tv;
   }
   else
   {
      aprs_stats[name].rx_sec += ((tv.tv_sec -
                aprs_stats[name].last_rx_sec.tv_sec) +
           (tv.tv_usec - aprs_stats[name].last_rx_sec.tv_usec)/1000000.0);
   }
} /* LocationInfo::setReceiving */


/****************************************************************************
 *
 * Protected member functions
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Private member functions
 *
 ****************************************************************************/

bool LocationInfo::parsePosition(const Config &cfg, const string &name)
{
  bool success = true;

  string pos_str(cfg.getValue(name,"LAT_POSITION"));
  if (!parseLatitude(loc_cfg.lat_pos, pos_str)) // weshalb wird loc_cfg benötigt (und nicht cfg??)
  {
    print_error(name, "LAT_POSITION", pos_str, "LAT_POSITION=51.20.10N");
    success = false;
  }

  pos_str = cfg.getValue(name,"LON_POSITION");
  if (!parseLongitude(loc_cfg.lon_pos, pos_str))
  {
    print_error(name, "LON_POSITION", pos_str, "LON_POSITION=12.10.30E");
    success = false;
  }

  return success;

} /* LocationInfo::parsePosition */


bool LocationInfo::parseLatitude(Coordinate &pos, const string &value)
{
  unsigned int deg, min, sec;
  char dir, sep[2];
  stringstream ss(value);

  ss >> deg >> sep[0] >> min >> sep[1] >> sec >> dir >> ws;

  if (ss.fail() || !ss.eof())
  {
    return false;
  }
  if (sep[0] != '.'|| sep[1] != '.')
  {
    return false;
  }
  if ((deg > 90) || (min > 59) || (sec > 59) ||
      ((deg == 90) && ((min > 0) || (sec > 0))))
  {
    return false;
  }
  if ((dir != 'N') && (dir != 'S'))
  {
    return false;
  }

  pos.deg = deg;
  pos.min = min;
  pos.sec = sec;
  pos.dir = dir;

  return true;

} /* LocationInfo::parseLatitude */


bool LocationInfo::parseLongitude(Coordinate &pos, const string &value)
{
  unsigned int deg, min, sec;
  char dir, sep[2];
  stringstream ss(value);

  ss >> deg >> sep[0] >> min >> sep[1] >> sec >> dir >> ws;

  if (ss.fail() || !ss.eof())
  {
    return false;
  }
  if (sep[0] != '.'|| sep[1] != '.')
  {
    return false;
  }
  if ((deg > 180) || (min > 59) || (sec > 59) ||
      ((deg == 180) && ((min > 0) || (sec > 0))))
  {
    return false;
  }
  if ((dir != 'E') && (dir != 'W'))
  {
    return false;
  }

  pos.deg = deg;
  pos.min = min;
  pos.sec = sec;
  pos.dir = dir;

  return true;

} /* LocationInfo::parseLongitude */


bool LocationInfo::parseStationHW(const Async::Config &cfg, const string &name)
{
  float frequency = 0;
  bool success = true;

  if (!cfg.getValue(name, "FREQUENCY", frequency))
  {
    print_error(name, "FREQUENCY", cfg.getValue(name, "FREQUENCY"),
                "FREQUENCY=438.875");
    success = false;
  }
  else
  {
    loc_cfg.frequency = lrintf(1000.0 * frequency);
  }

  if (!cfg.getValue(name, "TX_OFFSET", -9990, 9990, loc_cfg.tx_offset_khz, true))
  {
    print_error(name, "TX_OFFSET", cfg.getValue(name, "TX_OFFSET"),
                "TX_OFFSET=-600");
    success = false;
  }

  cfg.getValue(name, "NARROW", loc_cfg.narrow);

  if (!cfg.getValue(name, "TX_POWER", 1U, numeric_limits<unsigned int>::max(),
		    loc_cfg.power))
  {
    print_error(name, "TX_POWER", cfg.getValue(name, "TX_POWER"),
                "TX_POWER=8");
    success = false;
  }

  if (!cfg.getValue(name, "ANTENNA_GAIN", loc_cfg.gain, true))
  {
    print_error(name, "ANTENNA_GAIN", cfg.getValue(name, "ANTENNA_GAIN"),
                "ANTENNA_GAIN=6");
    success = false;
  }

  if (!parseAntennaHeight(loc_cfg, cfg.getValue(name, "ANTENNA_HEIGHT")))
  {
    print_error(name, "ANTENNA_HEIGHT", cfg.getValue(name, "ANTENNA_HEIGHT"),
                "ANTENNA_HEIGHT=10m");
    success = false;
  }

  if (!cfg.getValue(name, "ANTENNA_DIR", loc_cfg.beam_dir, true))
  {
    print_error(name, "ANTENNA_DIR", cfg.getValue(name, "ANTENNA_DIR"),
                "ANTENNA_DIR=-1");
    success = false;
  }

  if (!cfg.getValue(name, "TONE", loc_cfg.tone, true))
  {
    print_error(name, "TONE", cfg.getValue(name, "TONE"), "TONE=0");
    success = false;
  }

  int interval = 10;
  int max = numeric_limits<int>::max();
  if (!cfg.getValue(name, "BEACON_INTERVAL", 1, max, interval, true))
  {
    print_error(name, "BEACON_INTERVAL", cfg.getValue(name, "BEACON_INTERVAL"),
                "BEACON_INTERVAL=10");
    success = false;
  }
  else
  {
    loc_cfg.binterval = 60 * 1000 * interval;
  }

  loc_cfg.distance = 0.5;
  if (cfg.getValue(name, "NMEA_DISTANCE", loc_cfg.distance))
  {
    if (loc_cfg.distance < 0.5 || loc_cfg.distance > 10)
    {
      loc_cfg.distance = 0.5;
    }
  }

  if (!cfg.getValue(name, "NMEA_ANGLE", 1, 180, loc_cfg.angle, true))
  {
    loc_cfg.angle = 20;
  }
  loc_cfg.range = calculateRange(loc_cfg);

  return success;

} /* LocationInfo::parseStationHW */


bool LocationInfo::parsePath(const Async::Config &cfg, const string &name)
{
    // FIXME: Verify the path syntax!
    //        http://www.aprs.org/newN/new-eu-paradigm.txt
  loc_cfg.path = "WIDE1-1";
  cfg.getValue(name, "PATH", loc_cfg.path);
  return true;
} /* LocationInfo::parsePath */


int LocationInfo::calculateRange(const Cfg &cfg)
{
  float range_factor(1.0);

  if (cfg.range_unit == 'k')
  {
    range_factor = 1.60934;
  }

  double tmp = sqrt(2.0 * loc_cfg.height * sqrt((loc_cfg.power / 10.0) *
                        pow(10, loc_cfg.gain / 10.0) / 2)) * range_factor;

  return lrintf(tmp);

} /* LocationInfo::calculateRange */


bool LocationInfo::parseAntennaHeight(Cfg &cfg, const std::string value)
{
  char unit;
  unsigned int height;

  if (value.empty())
  {
    return true;
  }

  stringstream ss(value);
  ss >> height >> unit >> ws;
  if (ss.fail() || !ss.eof())
  {
    return false;
  }
  cfg.height = height;

  if (unit == 'm' || unit == 'M')
  {
      // Convert metric antenna height into feet
    cfg.height = lrintf(loc_cfg.height * 3.2808);
    cfg.range_unit = 'k';
  }

  return true;

} /* LocationInfo::parseAntennaHeight */


bool LocationInfo::parseClients(const Async::Config &cfg, const string &name)
{
  string aprs_server_list(cfg.getValue(name, "APRS_SERVER_LIST"));
  stringstream clientStream(aprs_server_list);
  string client, host;
  int port;
  bool success = true;

  while (clientStream >> client)
  {
    if (!parseClientStr(host, port, client))
    {
      print_error(name, "APRS_SERVER_LIST", aprs_server_list,
                  "APRS_SERVER_LIST=euro.aprs2.net:14580");
      success = false;
    }
    else
    {
      AprsTcpClient *client = new AprsTcpClient(loc_cfg, host, port);
      clients.push_back(client);
    }
  }

  clientStream.clear();
  string status_server_list(cfg.getValue(name, "STATUS_SERVER_LIST"));
  clientStream.str(status_server_list);
  while (clientStream >> client)
  {
    if (!parseClientStr(host, port, client))
    {
      print_error(name, "STATUS_SERVER_LIST", status_server_list,
                  "STATUS_SERVER_LIST=aprs.echolink.org:5199");
      success = false;
    }
    else
    {
      AprsUdpClient *client = new AprsUdpClient(loc_cfg, host, port);
      clients.push_back(client);
    }
  }

  return success;

} /* LocationInfo::parseClients */


bool LocationInfo::parseClientStr(string &host, int &port, const string &val)
{
  if (val.empty())
  {
    return false;
  }

  int tmpPort;
  string::size_type hostEnd;

  hostEnd = val.find_last_of(':');
  if (hostEnd == string::npos)
  {
    return false;
  }

  string portStr(val.substr(hostEnd+1, string::npos));
  stringstream ssval(portStr);
  ssval >> tmpPort;
  if (!ssval)
  {
    return false;
  }

  if (tmpPort < 0 || tmpPort > 0xffff)
  {
    return false;
  }

  host = val.substr(0, hostEnd);
  port = tmpPort;

  return true;

} /* LocationInfo::parseClientStr */


void LocationInfo::startStatisticsTimer(int sinterval)
{
  aprs_stats_timer.setTimeout(sinterval);
  aprs_stats_timer.setEnable(true);
} /* LocationInfo::statStatisticsTimer */


void LocationInfo::sendAprsStatistics(void)
{
  // https://github.com/PhirePhly/aprs_notes/blob/master/telemetry_format.md

    // FROM>APSVXn,VIA1,VIA2,VIAn:
  std::ostringstream addr;
  addr << loc_cfg.mycall << ">" << loc_cfg.destination << ":";

    // :ADDRESSEE:
  std::ostringstream addressee;
  addressee << ":" // << ":E" << loc_cfg.prefix << "-"
            << std::left << setw(9) << loc_cfg.mycall << ":";

  struct timeval now;
  gettimeofday(&now, NULL);

  bool send_metadata = ((now.tv_sec - last_tlm_metadata) > 1800);
  if (send_metadata)
  {
    last_tlm_metadata = now.tv_sec;

      // PARM.A1,A2,A3,A4,A5,B1,B2,B3,B4,B5,B6,B7,B8
    std::ostringstream parm;
    parm << addr.str()
         << addressee.str()
         << "PARM."
         << "RX Avg " << sinterval << "m"     // A1
         << ",TX Avg " << sinterval << "m"    // A2
         << ",RX Count " << sinterval << "m"  // A3
         << ",TX Count " << sinterval << "m"  // A4
         << ","                               // A5
         << ",RX"                             // B1
         << ",TX"                             // B2
         ;
    igateMessage(parm.str());

      // UNIT.A1,A2,A3,A4,A5,B1,B2,B3,B4,B5,B6,B7,B8
    std::ostringstream unit;
    unit << addr.str()
         << addressee.str()
         << "UNIT."
         << "erlang"          // A1
         << ",erlang"         // A2
         << ",receptions"     // A3
         << ",transmissions"  // A4
         ;
    igateMessage(unit.str());
  }

    // Loop for each logic
  for (auto& entry : LocationInfo::instance()->aprs_stats)
  {
    const std::string& logic_name = entry.first;
    AprsStatistics& stats = entry.second;

    if (!slogic.empty() && (logic_name != slogic))
    {
      continue;
    }

    if (stats.squelch_on)
    {
      setReceiving(logic_name, now, false);
    }
    if (stats.tx_on)
    {
      setTransmitting(logic_name, now, false);
    }

    const float erlang_b = 0.00392f;
    if (send_metadata)
    {
        // BITS.XXXXXXXX,Project Title
      std::ostringstream bits;
      bits << addr.str()
           << addressee.str()
           << "BITS.11111111,SvxLink " << logic_name;
      igateMessage(bits.str());

        // EQNS.a,b,c,a,b,c,a,b,c,a,b,c,a,b,c
      std::ostringstream eqns;
      eqns << addr.str()
           << addressee.str()
           << "EQNS."
           << "0"                                                   // A1a
           << "," << std::fixed << std::setprecision(5) << erlang_b // A1b
           << ",0"                                                  // A1c
           << ",0"                                                  // A2a
           << "," << std::fixed << std::setprecision(5) << erlang_b // A2b
           << ",0"                                                  // A2c
           ;
      igateMessage(eqns.str());
    }

      // T#nnn,nnn,nnn,nnn,nnn,nnn,nnnnnnnn
    auto rx_erlang = stats.rx_sec / (60.0f * sinterval);
    auto tx_erlang = stats.tx_sec / (60.0f * sinterval);
    std::ostringstream tlm;
    tlm << addr.str()
        << "T#" << std::setw(3) << std::setfill('0') << sequence
        << "," << std::setw(3) << std::setfill('0') << int(rx_erlang / erlang_b)
        << "," << std::setw(3) << std::setfill('0') << int(tx_erlang / erlang_b)
        << "," << std::setw(3) << std::setfill('0')
          << std::min(stats.rx_on_nr, 255U)
        << "," << std::setw(3) << std::setfill('0')
          << std::min(stats.tx_on_nr, 255U)
        << ",000"
        << "," << (stats.squelch_on ? 1 : 0)
        << (stats.tx_on ? 1 : 0)
        << "000000"
        ;
    igateMessage(tlm.str());

      // reset statistics if needed
    stats.reset();

    if (stats.squelch_on)
    {
      stats.rx_on_nr = 1;
      stats.last_rx_sec = now;
    }

    if (stats.tx_on)
    {
      stats.tx_on_nr = 1;
      stats.last_tx_sec = now;
    }

    if (++sequence > 999)
    {
      sequence = 0;
    }
  }
} /* LocationInfo::sendAprsStatistics */


void LocationInfo::initExtPty(std::string ptydevice)
{
  AprsPty *aprspty = new AprsPty();
  if (!aprspty->initialize(ptydevice))
  {
     cout << "*** ERROR: initializing aprs pty device " << ptydevice << endl;
  }
  else
  {
     aprspty->messageReceived.connect(mem_fun(*this,
                    &LocationInfo::mesReceived));
  }
} /* LocationInfo::initExtPty */


void LocationInfo::mesReceived(std::string message)
{
  string loc_call = LocationInfo::getCallsign();
  size_t found = message.find("XXXXXX");

  if (found != std::string::npos)
  {
    message.replace(found, 6, loc_call);
  }

  std::cout << "APRS PTY: " << message << std::endl;

  igateMessage(message);
} /* LocationInfo::mesReceived */


/****************************************************************************
 *
 * Private local functions
 *
 ****************************************************************************/

void LocationInfo::onNmeaReceived(char *buf, int count)
{
  nmeastream += buf;
  size_t found;
  while ((found = nmeastream.find("\n")) != string::npos)
  {
    if (found != 0)
    {
      handleNmea(nmeastream.substr(0, found));
    }
    nmeastream.erase(0, found+1);
  }
} /* LocationInfo::onNmeaReceived */


void LocationInfo::handleNmea(std::string message)
{
  //                $GPGLL,5119.48737,N,01201.09963,E,171526.00,A,A*6B
  //                $GPRMC,174403.00,A,5119.50974,N,01201.10715,E,0.182,,140422,,,A*7C
  //std::string reg = "GPGLL,[0-9]{3,}.[0-9]{2,},[NS],[0-9]{2,}.[0-9]{2,},[EW]";
  std::string reg = "GPRMC,[0-9]{3,}.[0-9]{1,},[AV],[0-9]{3,}.[0-9]{2,},[NS],[0-9]{3,}.[0-9]{2,},[EW],[0-9]{1,}.[0-9]{1,},";

  if (rmatch(message, reg))
  {
    getNextStr(message); // GPRMC
    getNextStr(message); // Time Stamp
    std::string valid = getNextStr(message); // validity A - ok, V - invalid
    if (valid[0] != 'A') return;
    std::string lat = getNextStr(message); // Latitude
    std::string ns = getNextStr(message); // N||S
    std::string lon = getNextStr(message); // Longitude
    std::string ew = getNextStr(message); // E||W
    position.speed = atof(getNextStr(message).c_str()); // speed in kts, 72.2
    position.track = atof(getNextStr(message).c_str()); // course, 231.5
    position.lat = atoi(lat.substr(0,2).c_str()) + atof(lat.substr(2,8).c_str())/60.0;
    if (ns[0] == 'S') position.lat *= -1.0;
    position.lon = atoi(lon.substr(0,3).c_str()) + atof(lon.substr(3,8).c_str())/60.0;
    if (ew[0] == 'W') position.lon *= -1.0;
    checkPosition();
    if (loc_cfg.debug)
    {
      cout << "NMEA: Latitude=" << lat << ", Longitude=" << lon << ", Speed="
           << position.speed << endl;
    }
  }
} /* LocationInfo::handleNmea */


void LocationInfo::checkPosition(void)
{
  float dist = calcDistance(position.lat, position.lon, stored_lat, stored_lon);
  float angle = calcAngle(position.lat, position.lon, stored_lat, stored_lon);
  if (dist >= loc_cfg.distance || angle >= loc_cfg.angle)
  {
    sendAprsPosition();
  }
  if (stored_lat == 0.0) stored_lat = position.lat;
  if (stored_lon == 0.0) stored_lon = position.lon;
} /* LocationInfo::checkPosition */


void LocationInfo::sendAprsPosition(void)
{
  if (position.lat >= 0)
  {
    loc_cfg.lat_pos.dir = 'N';
  }
  else
  {
    loc_cfg.lat_pos.dir = 'S';
  }
  if (position.lon >= 0)
  {
    loc_cfg.lon_pos.dir = 'E';
  }
  else
  {
    loc_cfg.lon_pos.dir = 'W';
  }

  loc_cfg.lat_pos.deg = int(abs(position.lat));
  loc_cfg.lat_pos.min = int(60*(abs(position.lat)-int(abs(position.lat))));
  loc_cfg.lat_pos.sec = 60*(60*(abs(position.lat)-int(abs(position.lat))) - loc_cfg.lat_pos.min);

  loc_cfg.lon_pos.deg = abs(int(position.lon));
  loc_cfg.lon_pos.min = int(60*(abs(position.lon)-int(abs(position.lon))));
  loc_cfg.lon_pos.sec = 60*(60*(abs(position.lon)-int(abs(position.lon))) - loc_cfg.lon_pos.min);

  stored_lat = position.lat;
  stored_lon = position.lon;

  ClientList::const_iterator it;
  for (it = clients.begin(); it != clients.end(); it++)
  {
    (*it)->sendBeacon();
  }
} /* LocationInfo::sendAprsMessage */


float LocationInfo::calcDistance(float lat1, float lon1, float lat2, float lon2)
{
  double dlon = M_PI * (lon2 - lon1) / 180.0;
  double dlat = M_PI * (lat2 - lat1) / 180.0;
  double a = (sin(dlat / 2) * sin(dlat / 2)) + cos(M_PI*lat1/180.0) *
              cos(M_PI*lat2/180) * (sin(dlon / 2) * sin(dlon / 2));
  double angle = 2 * atan2(sqrt(a), sqrt(1 - a));
  return static_cast<float>(static_cast<int>(angle * RADIUS * 100.))/100.;
} /* LocationInfo::calcDistance */


float LocationInfo::calcAngle(float lat1, float lon1, float lat2, float lon2)
{
  double x = cos(lat2) * sin(abs(lon1 - lon2));
  double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(abs(lon1 - lon2));
  return atan2(x, y);
} /* LocationInfo::calcAngle */


std::string LocationInfo::getNextStr(std::string& h)
{
  size_t f;
  std::string t = h.substr(0, f = h.find(','));
  if (f != std::string::npos)
  {
    h.erase(0, f + 1);
  }
  return t;
} /* LocationInfo::getNextStr */


bool LocationInfo::initNmeaDev(const Config &cfg, const std::string &name)
{
  std::string value = cfg.getValue(name, "NMEA_DEVICE");

  nmeadev = new Serial(value);
  if (!nmeadev->open(true))
  {
    cerr << "*** ERROR: Opening serial port NMEA_DEVICE="
         << value << endl;
    return false;
  }
  else
  {
    int baudrate = atoi(cfg.getValue(name, "NMEA_BAUD").c_str());
    if (baudrate != 2400 && baudrate != 4800 && baudrate != 9600)
    {
      cout << "+++ Setting default 4800 baud for " << name
           << "/NMEA_BAUD." << endl;
      baudrate = 4800; 
    }
    nmeadev->setParams(baudrate, Serial::PARITY_NONE, 8, 1, Serial::FLOW_NONE);
    nmeadev->charactersReceived.connect(
    	  mem_fun(*this, &LocationInfo::onNmeaReceived));

    cout << "Opening serial nmea device " << value << ", " 
         << baudrate << " Baud, version: " << NMEA_VERSION << endl;
  }
  return true;
} /* LocationInfo::initExtPty */


// needed by regex
bool LocationInfo::rmatch(std::string tok, std::string pattern)
{
  regex_t re;
  int status = regcomp(&re, pattern.c_str(), REG_EXTENDED);
  if (status != 0)
  {
    return false;
  }

  bool success = (regexec(&re, tok.c_str(), 0, NULL, 0) == 0);
  regfree(&re);
  return success;

} /* rmatch */


bool LocationInfo::initGpsdClient(const Config &cfg, const std::string &name)
{
  std::string value;
  bool debug(false);
  int port = 2947;
  std::string host = "localhost";
  if (cfg.getValue(name, "GPSD_PORT", value))
  {
    port = atoi(value.c_str());
  }

  if (cfg.getValue(name, "GPSD_HOST", value))
  {
    host = value;
  }
  
  cfg.getValue(name, "DEBUG", debug);

  cout << "Connecting Gpsd " << host << ":" << port << " (debug="
       << (debug ? "TRUE" : "FALSE") << "), version:"
       << NMEA_VERSION << endl;

  GpsdTcpClient *gpsdcl = new GpsdTcpClient(host, port, debug);
  gpsdcl->gpsdDataReceived.connect(mem_fun(*this,
                    &LocationInfo::gpsdDataReceived));
  return true;
} /* LocationInfo::initGpsdClient */


void LocationInfo::gpsdDataReceived(const Position ownpos)
{
  position = ownpos;
  checkPosition();
} /* LocationInfo::gpsdReceived */

  // Put all locally defined functions in an anonymous namespace to make them
  // file local. The "static" keyword has been deprecated in C++ so it
  // should not be used.
namespace
{

void print_error(const string &name, const string &variable,
                 const string &value, const string &example)
{
  cerr << "*** ERROR: Config variable [" << name << "]/" << variable << "="
       << value << " wrong or not set.";

  if (!example.empty())
  {
    cerr << "\n*** Example: " <<  example;
  }
  cerr << endl;
} /* print_error */

} // End of anonymous namespace


/*
 * This file has not been truncated
 */
