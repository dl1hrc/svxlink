/**
@file	 LocationInfo.h
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


#ifndef LOCATION_INFO_INCLUDED
#define LOCATION_INFO_INCLUDED


/****************************************************************************
 *
 * System Includes
 *
 ****************************************************************************/

#include <string>
#include <vector>
#include <list>
#include <sys/time.h>


/****************************************************************************
 *
 * Project Includes
 *
 ****************************************************************************/

#include <AsyncConfig.h>
#include <AsyncTimer.h>
#include <EchoLinkStationData.h>


/****************************************************************************
 *
 * Local Includes
 *
 ****************************************************************************/

#include "AprsPty.h"


/****************************************************************************
 *
 * Forward declarations
 *
 ****************************************************************************/

class AprsClient;


/****************************************************************************
 *
 * Namespace
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Forward declarations of classes inside of the declared namespace
 *
 ****************************************************************************/




/****************************************************************************
 *
 * Defines & typedefs
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Exported Global Types
 *
 ****************************************************************************/


/****************************************************************************
 *
 * Class definitions
 *
 ****************************************************************************/

class LocationInfo
{
  public:
    static LocationInfo* instance()
    {
       if (!_instance)
        return NULL; // illegal pointer, if not initialized
       return _instance;
    }

    static bool has_instance()
    {
        return _instance;
    }

    static void deleteInstance(void)
    {
      delete _instance;
      _instance = 0;
    }

    struct Coordinate
    {
      unsigned int  deg {0};
      unsigned int  min {0};
      unsigned int  sec {0};
      char          dir {'N'};
    };

    LocationInfo(void);
    LocationInfo(const LocationInfo&) = delete;

    std::string getCallsign();

    struct AprsStatistics
    {
      unsigned    rx_on_nr        {0};
      unsigned    tx_on_nr        {0};
      float       rx_sec          {0.0f};
      float       tx_sec          {0.0f};
      struct timeval last_rx_sec  {0, 0};
      struct timeval last_tx_sec  {0, 0};
      bool        tx_on           {false};
      bool        squelch_on      {false};

      void reset(void)
      {
        rx_on_nr = 0;
        tx_on_nr = 0;
        rx_sec = 0.0f;
        tx_sec = 0.0f;
        last_rx_sec = {0, 0};
        last_tx_sec = {0, 0};
      }
    };

    using aprs_struct = std::map<std::string, AprsStatistics>;
    aprs_struct aprs_stats;

    struct Cfg
    {
      unsigned int binterval    {10}; // Minutes
      unsigned int frequency    {0};
      unsigned int power        {0};
      unsigned int tone         {0};
      unsigned int height       {10};
      unsigned int gain         {0};
      int          beam_dir     {-1};
      unsigned int range        {0};
      char         range_unit   {'m'};

      Coordinate  lat_pos;
      Coordinate  lon_pos;

      std::string logincall;
      std::string loginssid;
      std::string mycall;
      std::string prefix;
      std::string path          {"TCPIP*"};
      std::string comment       {"SvxLink by SM0SVX (www.svxlink.org)"};
      std::string destination   {"APSVX1"};
      bool        debug         {false};
      std::string filter;
      std::string symbol        {"S0"};
      int         tx_offset_khz {0};
      bool        narrow        {false};
    };

    static bool initialize(Async::Config& cfg, const std::string& cfg_name);

    void updateDirectoryStatus(EchoLink::StationData::Status new_status);
    void igateMessage(const std::string& info);
    void update3rdState(const std::string& call, const std::string& info);
    void updateQsoStatus(int action, const std::string& call,
                         const std::string& name,
			 std::list<std::string>& call_list);
    bool getTransmitting(const std::string &name);
    void setTransmitting(const std::string &name, struct timeval tv,
                         bool is_transmitting);
    void setReceiving(const std::string &name, struct timeval tv,
                      bool is_receiving);

  private:
    static LocationInfo* _instance;

    typedef std::list<AprsClient*> ClientList;

    Cfg           loc_cfg; // weshalb?
    ClientList    clients;
    int           sequence          {0};
    Async::Timer  aprs_stats_timer  {-1, Async::Timer::TYPE_PERIODIC};
    unsigned int  sinterval         {10}; // Minutes
    std::string   slogic;
    time_t        last_tlm_metadata {0};

    bool parsePosition(const Async::Config &cfg, const std::string &name);
    bool parseLatitude(Coordinate &pos, const std::string &value);
    bool parseLongitude(Coordinate &pos, const std::string &value);

    bool parseStationHW(const Async::Config &cfg, const std::string &name);
    bool parsePath(const Async::Config &cfg, const std::string &name);
    int calculateRange(const Cfg &cfg);
    bool parseAntennaHeight(Cfg &cfg, const std::string value);
    bool parseClientStr(std::string &host, int &port, const std::string &val);
    bool parseClients(const Async::Config &cfg, const std::string &name);
    void startStatisticsTimer(int sinterval);
    void sendAprsStatistics(void);
    void initExtPty(std::string ptydevice);
    void mesReceived(std::string message);

};  /* class LocationInfo */

#endif /* LOCATION_INFO_INCLUDED */

/*
 * This file has not been truncated
 */
