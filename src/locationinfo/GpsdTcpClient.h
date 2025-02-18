/**
@file	 GpsdTcpClient.h
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


#ifndef GPSD_TCP_CLIENT
#define GPSD_TCP_CLIENT


/****************************************************************************
 *
 * System Includes
 *
 ****************************************************************************/

#include <string>
#include <vector>


/****************************************************************************
 *
 * Project Includes
 *
 ****************************************************************************/

#include <AsyncTcpClient.h>


/****************************************************************************
 *
 * Local Includes
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Forward declarations
 *
 ****************************************************************************/

namespace Async
{
  class Timer;
};


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

typedef struct {
  double lat;
  double lon;
  float climbrate;
  float speed;
  float altitude;
  float track;
} Position;



/****************************************************************************
 *
 * Exported Global Variables
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Class definitions
 *
 ****************************************************************************/


/**
@brief	Gpsd-logics
@author Adi Bier / DL1HRC
@date   2008-11-01
*/
class GpsdTcpClient : public sigc::trackable
{
  public:
     GpsdTcpClient(const std::string &server, int port, bool debug);

     ~GpsdTcpClient(void);

    /**
     * @brief 	A signal that is emitted when a valid gps position is send
     */
    sigc::signal<void, Position>       	gpsdDataReceived;

  private:

    std::string		server;
    int			port;
    bool		debug;
    Async::TcpClient<>* con;
    Async::Timer        *reconnect_timer;

    typedef std::vector<std::string> StrList;
    StrList  gpsdparams;

    void sendMsg(const char *msg);
    void  tcpConnected(void);
    int   tcpDataReceived(Async::TcpClient<>::TcpConnection *con, void *buf,
                          int count);
    void  tcpDisconnected(Async::TcpClient<>::TcpConnection *con,
                          Async::TcpClient<>::DisconnectReason reason);
    void reconnectGpsd(Async::Timer *t);
    void pollTimeout(Async::Timer *t);

};  /* class GpsdTcpClient */


#endif /* GPSD_TCP_CLIENT */

/*
 * This file has not been truncated
 */
