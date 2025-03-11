/**
@file	 AprsTcpClient.cpp
@brief   Contains an implementation of APRS updates via TCP
@author  Adi Bier / DL1HRC
@date	 2008-12-01

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

#include <iostream>
#include <regex>
#include <cmath>
#include <cstring>
#include <ctime>


/****************************************************************************
 *
 * Project Includes
 *
 ****************************************************************************/

#include <AsyncTimer.h>
#include <AsyncConfig.h>


/****************************************************************************
 *
 * Local Includes
 *
 ****************************************************************************/

#include "version/SVXLINK.h"
#include "AprsTcpClient.h"
#include "AprsUdpClient.h"
#include "common.h"


/****************************************************************************
 *
 * Namespaces to use
 *
 ****************************************************************************/

using namespace std;
using namespace Async;
using namespace SvxLink;


/****************************************************************************
 *
 * Defines & typedefs
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Local class definitions
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Prototypes
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Exported Global Variables
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Local Global Variables
 *
 ****************************************************************************/

#define HASH_KEY	0x73e2 			// This is the seed for the key

namespace {
    // Extended frequency mapping according to UNOFFICIAL APRS Protocol
    // Reference 1.2 Chapter 18.
    // A96.000MHz would be 1296 MHz
    // B20.000MHz would be 2320 MHz
    // C01.000MHz would be 2401 MHz
    // D01.000MHz would be 3401 MHz
    // E51.000MHz would be 5651 MHz
    // F60.000MHz would be 5760 MHz
    // G30.000MHz would be 5830 MHz
    // H01.000MHz would be 10,101 MHz
    // I01.000MHz would be 10,201 MHz
    // J68.000MHz would be 10,368 MHz
    // K01.000MHz would be 10,401 MHz
    // L01.000MHz would be 10,501 MHz
    // M48.000MHz would be 24,048 MHz
    // N01.000MHz would be 24,101 MHz
    // O01.000MHz would be 24,201 MHz
  const std::map<unsigned, char> freq_map{
    {12, 'A'},
    {23, 'B'},
    {24, 'C'},
    {34, 'D'},
    {56, 'E'},
    {57, 'F'},
    {58, 'G'},
    {101, 'H'},
    {102, 'I'},
    {103, 'J'},
    {104, 'K'},
    {105, 'L'},
    {240, 'M'},
    {241, 'N'},
    {242, 'O'}
  };
};

/****************************************************************************
 *
 * Public member functions
 *
 ****************************************************************************/

AprsTcpClient::AprsTcpClient(LocationInfo::Cfg &loc_cfg,
                            const std::string &server, int port)
  : loc_cfg(loc_cfg), server(server), port(port), con(0), beacon_timer(0),
    reconnect_timer(0), offset_timer(0), num_connected(0)
{
   StrList str_list;

   con = new TcpClient<>(server, port);
   con->connected.connect(mem_fun(*this, &AprsTcpClient::tcpConnected));
   con->disconnected.connect(mem_fun(*this, &AprsTcpClient::tcpDisconnected));
   con->dataReceived.connect(mem_fun(*this, &AprsTcpClient::tcpDataReceived));
   con->connect();

   beacon_timer = new Timer(loc_cfg.binterval * 60 * 1000, Timer::TYPE_PERIODIC);
   beacon_timer->setEnable(false);
   beacon_timer->expired.connect(mem_fun(*this, &AprsTcpClient::sendAprsBeacon));

   offset_timer = new Timer(10000, Timer::TYPE_ONESHOT);
   offset_timer->setEnable(false);
   offset_timer->expired.connect(mem_fun(*this,
                 &AprsTcpClient::startNormalSequence));

   reconnect_timer = new Timer(5000);
   reconnect_timer->setEnable(false);
   reconnect_timer->expired.connect(mem_fun(*this,
                 &AprsTcpClient::reconnectAprsServer));
} /* AprsTcpClient::AprsTcpClient */


AprsTcpClient::~AprsTcpClient(void)
{
   delete con;
   delete reconnect_timer;
   delete offset_timer;
   delete beacon_timer;
} /* AprsTcpClient::~AprsTcpClient */


void AprsTcpClient::updateQsoStatus(int action, const string& call,
  const string& info, list<string>& call_list)
{
  if (loc_cfg.prefix.empty())
  {
    return;
  }

  num_connected = call_list.size();

  char msg[80];
  switch(action)
  {
    case 0:
      sprintf(msg, "connection to %s closed", call.c_str());
      break;
    case 1:
      sprintf(msg, "connection to %s (%s)", call.c_str(), info.c_str());
      break;
    case 2:
      sprintf(msg, "incoming connection %s (%s)", call.c_str(), info.c_str());
      break;
  }

    // Object message for Echolink
    // ;EL-242660*111111z4900.05NE00823.29E0QSO status message
  std::ostringstream objmsg;
  objmsg << addrStr()
         << ";" << addresseeStr(prefixStr() + loc_cfg.mycall) << "*"
         << "111111z"
         << posStr()
         << msg
         ;
  sendMsg(objmsg.str());

    // Status message for Echolink, connected calls
  std::string status = prefixStr() + addrStr() + ">";
  for (const auto& call : call_list)
  {
    status += call + " ";
  }
  sendMsg(status);
} /* AprsTcpClient::updateQsoStatus */


// updates state of a 3rd party
void AprsTcpClient::update3rdState(const string& call, const string& info)
{
   char aprsmsg[20 + info.length()];
   sprintf(aprsmsg, "%s>%s", call.c_str(), info.c_str());
   sendMsg(aprsmsg);
} /* AprsTcpClient::update3rdState */


void AprsTcpClient::igateMessage(const string& info)
{
  sendMsg(info);
} /* AprsTcpClient::igateMessage */


void AprsTcpClient::sendBeacon(void)
{
  sendABeacon();
} /* AprsTcpClient::sendBeacon */



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

std::string AprsTcpClient::addrStr(void)
{
    // MYCALL>APSVXn,path:
  std::string addr = loc_cfg.mycall + ">" + loc_cfg.destination;
  if (!loc_cfg.path.empty())
  {
    addr += std::string(",") + loc_cfg.path;
  }
  addr += ":";
  return addr;
} /* AprsTcpClient::addrStr */


std::string AprsTcpClient::posStr(const std::string& symbol)
{
  char symbol_table_id = 'E';
  char symbol_code = '0';

  if (symbol.size() >= 2)
  {
    symbol_table_id = symbol[0];
    symbol_code = symbol[1];
  }
  else if (num_connected > 0)
  {
      // Set overlay
    symbol_table_id = (num_connected < 10) ? '0' + num_connected : '9';
  }

  char pos[32];
  sprintf(pos, "%02d%02d.%02d%c%c%03d%02d.%02d%c%c",
               loc_cfg.lat_pos.deg, loc_cfg.lat_pos.min,
               (loc_cfg.lat_pos.sec * 100) / 60, loc_cfg.lat_pos.dir,
               symbol_table_id,
               loc_cfg.lon_pos.deg, loc_cfg.lon_pos.min,
               (loc_cfg.lon_pos.sec * 100) / 60, loc_cfg.lon_pos.dir,
               symbol_code);
  return std::string(pos);
} /* AprsTcpClient::posStr */


std::string AprsTcpClient::timeStr(void)
{
  time_t now = time(NULL);
  struct tm tm, *tm_ptr;
  tm_ptr = gmtime_r(&now, &tm);
  std::ostringstream tstr;
  tstr << std::setw(2) << std::setfill('0') << tm_ptr->tm_mday
       << std::setw(2) << std::setfill('0') << tm_ptr->tm_hour
       << std::setw(2) << std::setfill('0') << tm_ptr->tm_min
       << "z"
       ;
  return tstr.str();
} /* AprsTcpClient::timeStr */


std::string AprsTcpClient::phgStr(void)
{
  return AprsUdpClient::phgStr(loc_cfg.power, loc_cfg.height, loc_cfg.gain,
                               loc_cfg.beam_dir);
} /* AprsTcpClient::phgStr */


std::string AprsTcpClient::toneStr(void)
{
    // CTCSS/1750Hz tone
  char tone[5];
  if ((loc_cfg.tone == 0) || (loc_cfg.tone > 9999))
  {
    sprintf(tone, "%coff", (loc_cfg.narrow ? 't' : 'T'));
  }
  else if (loc_cfg.tone < 1000)
  {
    sprintf(tone, "%c%03d", (loc_cfg.narrow ? 't' : 'T'), loc_cfg.tone);
  }
  else if (loc_cfg.tone < 2000)
  {
    sprintf(tone, "%c%03d", (loc_cfg.narrow ? 'l' : '1'), loc_cfg.tone - 1000);
  }
  else
  {
    sprintf(tone, "%04d", loc_cfg.tone);
  }
  return tone;
} /* AprsTcpClient::toneStr */


void AprsTcpClient::sendABeacon(void)
{
  const std::string tx_offset_str = prependSpaceIfNotEmpty(txOffsetStr());
  const std::string comment_str = prependSpaceIfNotEmpty(loc_cfg.comment);

  if (!loc_cfg.prefix.empty())
  {
      // Object message for Echolink
    std::ostringstream objmsg;
    objmsg << addrStr()
           << ";" << addresseeStr(prefixStr() + loc_cfg.mycall) << "*"
           << "111111z"
           << posStr()
           << phgStr()
           << "/" << frequencyStr()
           << " " << toneStr()
           << tx_offset_str
           << " " << rangeStr()
           << comment_str;
    sendMsg(objmsg.str());
  }

    // Position report for main callsign
  std::ostringstream posmsg;
  posmsg << addrStr()
         << "="
         << posStr(loc_cfg.symbol)
         << phgStr()
         << "/" << frequencyStr()
         << " " << toneStr()
         << tx_offset_str
         << " " << rangeStr()
         << comment_str;
  sendMsg(posmsg.str());
} /* AprsTcpClient::sendABeacon */


std::string AprsTcpClient::txOffsetStr(void)
{
  std::ostringstream offset;
  if (std::abs(loc_cfg.tx_offset_khz) <= 9990)
  {
    offset << std::showpos << std::setw(4) << std::internal
           << (loc_cfg.tx_offset_khz / 10);
  }
  return offset.str();
} /* AprsTcpClient::txOffsetStr */


std::string AprsTcpClient::frequencyStr(void)
{
  std::ostringstream fq;
  unsigned freq_khz = loc_cfg.frequency;
  const unsigned mhz100 = freq_khz / 100000;
  if (mhz100 > 9)
  {
    auto it = freq_map.find(mhz100);
    if (it != freq_map.end())
    {
      fq << it->second;
    }
    else
    {
      freq_khz = 0;
      fq << "0";
    }
  }
  else
  {
    fq << mhz100;
  }
  fq << std::fixed << std::setw(6) << std::setfill('0') << std::setprecision(3)
     << ((freq_khz % 100000) / 1000.0)
     << "MHz";
  return fq.str();
} /* AprsTcpClient::frequencyStr */


std::string AprsTcpClient::rangeStr(void)
{
  std::ostringstream range;
  range << "R" << std::setw(2) << loc_cfg.range << loc_cfg.range_unit;
  return range.str();
} /* AprsTcpClient::rangeStr */


std::string AprsTcpClient::prefixStr(void) const
{
  return std::string("E") + loc_cfg.prefix + "-";
} /* AprsTcpClient::prefixStr */


std::string AprsTcpClient::addresseeStr(const std::string& call)
{
  std::ostringstream addressee;
  addressee << std::left << std::setw(9) << call;
  return addressee.str();
} /* AprsTcpClient::addresseeStr */


std::string AprsTcpClient::prependSpaceIfNotEmpty(const std::string& str)
{
  return (str.empty() ? std::string() : std::string(" ").append(str));
} /* AprsTcpClient::prependSpaceIfNotEmpty */


void AprsTcpClient::sendAprsBeacon(Timer *t)
{
  sendABeacon();
} /* AprsTcpClient::sendABeacon */


void AprsTcpClient::sendMsg(std::string aprsmsg)
{
  const size_t max_packet_size = 512;
  if (aprsmsg.size() >= max_packet_size-2)
  {
    std::cerr << "*** WARNING: APRS packet too long (>= "
              << max_packet_size << " bytes): "
              << aprsmsg
              << std::endl;
    return;
  }

  if (loc_cfg.debug)
  {
    std::cout << "APRS: " << aprsmsg << std::endl;
  }

  if (!con->isConnected() || aprsmsg.empty())
  {
    return;
  }

  aprsmsg.append("\r\n");

  int written = con->write(aprsmsg.c_str(), aprsmsg.size());
  if (written < 0)
  {
    cerr << "*** ERROR: TCP write error" << endl;
  }
  else if (static_cast<size_t>(written) != aprsmsg.size())
  {
    cerr << "*** ERROR: TCP transmit buffer overflow, reconnecting." << endl;
    con->disconnect();
  }
} /* AprsTcpClient::sendMsg */


void AprsTcpClient::aprsLogin(void)
{
  const auto& logincall = loc_cfg.logincall;
  const auto& loginssid = loc_cfg.loginssid;
  std::ostringstream loginmsg;
  loginmsg << "user " << (logincall + loginssid)
           << " pass " << getPasswd(logincall)
           << " vers SvxLink " << SVXLINK_VERSION;
  if (!loc_cfg.filter.empty())
  {
    loginmsg << " filter " << loc_cfg.filter;
  }
  sendMsg(loginmsg.str());
} /* AprsTcpClient::aprsLogin */


// generate passcode for the aprs-servers, copied from xastir-source...
// special tnx to:
// Copyright (C) 1999,2000  Frank Giannandrea
// Copyright (C) 2000-2008  The Xastir Group

short AprsTcpClient::getPasswd(const string& call)
{
  short hash = HASH_KEY;
  string::size_type i = 0;
  string::size_type len = call.length();
  const char *ptr = call.c_str();

  while (i < len)
  {
    hash ^= toupper(*ptr++) << 8;
    hash ^= toupper(*ptr++);
    i += 2;
  }

  return (hash & 0x7fff);
} /* AprsTcpClient::callpass */


void AprsTcpClient::tcpConnected(void)
{
  cout << "Connected to APRS server " << con->remoteHost() <<
          " on port " << con->remotePort() << endl;

  recv_buf.clear();

  aprsLogin();                    // login
  offset_timer->reset();          // reset the offset timer
  offset_timer->setEnable(true);  // restart the offset timer
} /* AprsTcpClient::tcpConnected */


void AprsTcpClient::startNormalSequence(Timer *t)
{
  beacon_timer->setEnable(true);		// start the beaconinterval
} /* AprsTcpClient::startNormalSequence */


void AprsTcpClient::decodeAprsPacket(std::string frame)
{
  if (frame.at(frame.size() - 1) == '\r')
  {
    frame.erase(frame.size() - 1);
  }
  if (frame.empty())
  {
    return;
  }
  if (loc_cfg.debug)
  {
    if (frame.at(0) != '#')
    {
      std::cout << "APRS: " << frame << std::endl;
    }
  }

  std::smatch m;

    // Ex: "ISS>APWW11,KJ4ERJ-15*,TCPIP*,qAS,KJ4ERJ-15:"
  const std::regex msg_addr("([^>]{1,9})>([^:,]{1,9})((?:,[^:,]{1,10})*):(.*)");
  if (!std::regex_match(frame, m, msg_addr) && (m.size() != 5))
  {
    return;
  }
  std::string from{m[1]};
  std::string to{m[2]};
  std::string via = m[3];
  std::string aprs_msg{m[4]};
  //std::cout << "### from='" << from << "'"
  //          << " to='" << to << "'"
  //          << " via='" << via << "'"
  //          << " aprs_msg='" << aprs_msg << "'"
  //          << std::endl;

    // Ex: ":SM0SVX   :AOS 4h41m (20 0226z) SE^4{RT}"
  const std::regex msg_re(":(.{9}):([^|~{]{0,67})(?:\\{(.{1,5}))?");
  if (std::regex_match(aprs_msg, m, msg_re) && (m.size() >= 3))
  {
    std::string addressee = m[1];
    std::string text = m[2];
    std::string id;
    if (m.size() == 4)
    {
      id = m[3];
    }
    auto spacepos = addressee.find(' ');
    if (spacepos != std::string::npos)
    {
      addressee.erase(spacepos);
    }
    if (addressee == loc_cfg.mycall)
    {
      std::cout << "APRS message";
      if (!id.empty())
      {
        std::cout << "[" << id << "]";
      }
      std::cout << " from " << from << ":"
                << " " << text
                << std::endl;

      if (!id.empty())
      {
        std::ostringstream ack;
        ack << loc_cfg.mycall << ">" << loc_cfg.destination
            << "::" << addresseeStr(from) << ":ack" << id;
        sendMsg(ack.str());
      }
    }
  }

} /* AprsTcpClient::decodeAprsPacket */


int AprsTcpClient::tcpDataReceived(TcpClient<>::TcpConnection *con,
                                   void *buf, int count)
{
  auto orig_count = count;
  const char* first = reinterpret_cast<const char*>(buf);
  while (count > 0)
  {
    const char* nl = reinterpret_cast<const char*>(memchr(first, '\n', count));
    if (nl != nullptr)
    {
      recv_buf.append(first, nl);
      if (!recv_buf.empty())
      {
        decodeAprsPacket(recv_buf);
        recv_buf.clear();
      }
      count -= nl - first + 1;
      first = nl+1;
    }
    else
    {
      recv_buf.append(first, count);
    }
  }

  return orig_count;
} /* AprsTcpClient::tcpDataReceived */



void AprsTcpClient::tcpDisconnected(TcpClient<>::TcpConnection *con,
                                    TcpClient<>::DisconnectReason reason)
{
  cout << "*** WARNING: Disconnected from APRS server" << endl;

  beacon_timer->setEnable(false);		// no beacon while disconnected
  reconnect_timer->setEnable(true);		// start the reconnect-timer
  offset_timer->setEnable(false);
  offset_timer->reset();
  recv_buf.clear();
} /* AprsTcpClient::tcpDisconnected */


void AprsTcpClient::reconnectAprsServer(Async::Timer *t)
{
  reconnect_timer->setEnable(false);		// stop the reconnect-timer
  cout << "*** WARNING: Trying to reconnect to APRS server" << endl;
  con->connect();
} /* AprsTcpClient::reconnectNextAprsServer */



/*
 * This file has not been truncated
 */
