/**
@file	 GpadTcpClient.cpp
@brief   Contains an client to fetch data from the gpsd
@author  Adi Bier / DL1HRC
@date	 2022-04-13

\verbatim
SvxLink - A Multi Purpose Voice Services System for Ham Radio Use
Copyright (C) 2003-2022 Tobias Blomberg / SM0SVX

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
#include <cmath>
#include <cstring>
#include <sys/time.h>


/****************************************************************************
 *
 * Project Includes
 *
 ****************************************************************************/

#include <AsyncTimer.h>


/****************************************************************************
 *
 * Local Includes
 *
 ****************************************************************************/

#include "version/SVXLINK.h"
#include "GpsdTcpClient.h"
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

 #define GPSD_VERSION "12112025"


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



/****************************************************************************
 *
 * Public member functions
 *
 ****************************************************************************/

GpsdTcpClient::GpsdTcpClient(const std::string &server, int port, bool debug)
  : server(server), port(port), debug(debug), con(0), reconnect_timer(0)
{
   con = new TcpClient<>(server, port);
   con->connected.connect(mem_fun(*this, &GpsdTcpClient::tcpConnected));
   con->disconnected.connect(mem_fun(*this, &GpsdTcpClient::tcpDisconnected));
   con->dataReceived.connect(mem_fun(*this, &GpsdTcpClient::tcpDataReceived));
   con->connect();

   reconnect_timer = new Timer(5000);
   reconnect_timer->setEnable(false);
   reconnect_timer->expired.connect(mem_fun(*this,
                 &GpsdTcpClient::reconnectGpsd));

} /* GpsdTcpClient::GpsdTcpClient */


GpsdTcpClient::~GpsdTcpClient(void)
{
   delete con;
   delete reconnect_timer;
} /* GpsdTcpClient::~GpsdTcpClient */


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

void GpsdTcpClient::sendMsg(const char *msg)
{

  if (!con->isConnected())
  {
    return;
  }

  if (debug)
  {
    cout << msg << endl;
  }

  int written = con->write(msg, strlen(msg));
  if (written < 0)
  {
    cerr << "*** ERROR: TCP write error" << endl;
  }
  else if ((size_t)written != strlen(msg))
  {
    cerr << "*** ERROR: TCP transmit buffer overflow, reconnecting." << endl;
    con->disconnect();
  }
} /* GpsdTcpClient::sendMsg */


void GpsdTcpClient::tcpConnected(void)
{
  cout << "Connected to Gpsd " << con->remoteHost() <<
          " on port " << con->remotePort() << endl;

  // start 1st message with watch=enable,json=true
  sendMsg("?WATCH={\"enable\":true,\"json\":true}\r");
} /* GpsdTcpClient::tcpConnected */


// Incoming Tcp messages from the Gpsd
int GpsdTcpClient::tcpDataReceived(TcpClient<>::TcpConnection *con,
                                   void *buf, int count)
{
  /*
  {
    "class":"POLL",
    "time":"2021-09-02T11:20:23.008Z",
    "active":1,
    "tpv":[{"class":"TPV","device":"/dev/ttyACM0",
    "mode":3,
    "time":"2021-09-02T11:20:22.000Z",
    "ept":0.005,
    "lat":51.325000500,
    "lon":12.018431667,
    "altHAE":155.700,"altMSL":110.700,"alt":110.700,
    "epx":10.902,"epy":8.834,"epv":25.990,"magvar":3.6,"speed":0.001,
    "climb":-0.100,"eps":21.80,"epc":51.98,"geoidSep":45.000,
    "eph":17.860,"sep":27.930}],"gst":[{"class":"GST","device":"/dev/ttyACM0"}],
    "sky":[
    {"class":"SKY","device":"/dev/ttyACM0","xdop":0.73,"ydop":0.59,
    "vdop":1.03,"tdop":0.73,"hdop":0.94,"gdop":1.57,"pdop":1.39,
    "satellites":
    [
    {"PRN":1,"el":89.0,"az":302.0,"ss":19.0,"used":true,"gnssid":0,"svid":1},
    {"PRN":3,"el":58.0,"az":252.0,"ss":46.0,"used":true,"gnssid":0,"svid":3},
    {"PRN":4,"el":11.0,"az":193.0,"ss":37.0,"used":true,"gnssid":0,"svid":4},
    {"PRN":8,"el":14.0,"az":177.0,"ss":35.0,"used":true,"gnssid":0,"svid":8},
    {"PRN":14,"el":10.0,"az":272.0,"ss":24.0,"used":true,"gnssid":0,"svid":14},
    {"PRN":17,"el":36.0,"az":306.0,"ss":42.0,"used":true,"gnssid":0,"svid":17},
    {"PRN":19,"el":14.0,"az":322.0,"ss":21.0,"used":true,"gnssid":0,"svid":19},
    {"PRN":21,"el":67.0,"az":130.0,"ss":31.0,"used":true,"gnssid":0,"svid":21},
    {"PRN":22,"el":82.0,"az":261.0,"ss":30.0,"used":true,"gnssid":0,"svid":22},
    {"PRN":28,"el":15.0,"az":291.0,"ss":19.0,"used":false,"gnssid":0,"svid":28},
    {"PRN":31,"el":10.0,"az":104.0,"ss":0.0,"used":false,"gnssid":0,"svid":31},
    {"PRN":32,"el":29.0,"az":50.0,"ss":25.0,"used":true,"gnssid":0,"svid":32}
    ]}]
  } */
  Position pos;

  std::stringstream ss;
  ss.write(reinterpret_cast<const char*>(buf), count);
  std::string s = ss.str();
  size_t found;
  short mode = 0;

   // split the Gpsd message
  splitStr(gpsdparams, s, ",");
  for (StrList::const_iterator it = gpsdparams.begin(); it != gpsdparams.end(); it++)
  {
    std::string s = *it;
    if ((found = s.find("\"mode\":")) != string::npos)
    {
      mode = atoi(s.erase(0,7).c_str());
    }
    if ((found = s.find("\"altMSL\":")) != string::npos)
    {
      pos.altitude = atof(s.erase(0,9).c_str());
    }
    if ((found = s.find("\"lon\":")) != string::npos)
    {
      pos.lon = atof(s.erase(0,6).c_str());
    }
    if ((found = s.find("\"lat\":")) != string::npos)
    {
      pos.lat = atof(s.erase(0,6).c_str());
    }
    if ((found = s.find("\"climb\":")) != string::npos)
    {
      pos.climbrate = atof(s.erase(0,8).c_str());
    }
    if ((found = s.find("\"speed\":")) != string::npos)
    {
      pos.speed = atof(s.erase(0,8).c_str());
    }
  }

  // mode: 0==unknown, 1==no fix, 2==2D, 3==3D
  if (mode == 2 || mode == 3)
  {
    gpsdDataReceived(pos);
    if (debug)
    {
      cout << "gpsd: mode=" << mode << "D, " << setprecision(9)
           << "Latitude=" << pos.lat << " Longitude=" << pos.lon << " Speed="
           << setprecision(1) << pos.speed << ", Altitude=" << pos.altitude
           << endl;
    }
  }
  return count;                                // do nothing...
} /* GpsdTcpClient::tcpDataReceived */


void GpsdTcpClient::tcpDisconnected(TcpClient<>::TcpConnection *con,
                                    TcpClient<>::DisconnectReason reason)
{
  cout << "*** WARNING: Disconnected from Gpsd" << endl;
  reconnect_timer->setEnable(true);		// start the reconnect-timer
} /* GpsdTcpClient::tcpDisconnected */


void GpsdTcpClient::reconnectGpsd(Async::Timer *t)
{
  reconnect_timer->setEnable(false);		// stop the reconnect-timer
  cout << "*** WARNING: Trying to reconnect to Gpsd server" << endl;
  con->connect();
} /* GpsdTcpClient::reconnectGpsd */


/*
 * This file has not been truncated
 */
