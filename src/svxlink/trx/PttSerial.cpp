/**
@file	 PttSerial.cpp
@brief   A PTT hardware controller using a Serial to signal an external script
@author  Tobias Blomberg / SM0SVX & Steve Koehler / DH1DM & Adi Bier / DL1HRC
@date	 2024-10-01

\verbatim
SvxLink - A Multi Purpose Voice Services System for Ham Radio Use
Copyright (C) 2003-2024 Tobias Blomberg / SM0SVX

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
#include <cstring>


/****************************************************************************
 *
 * Project Includes
 *
 ****************************************************************************/


#include "common.h"

/****************************************************************************
 *
 * Local Includes
 *
 ****************************************************************************/

#include "PttSerial.h"



/****************************************************************************
 *
 * Namespaces to use
 *
 ****************************************************************************/

using namespace std;
using namespace sigc;
using namespace Async;


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



/****************************************************************************
 *
 * Public member functions
 *
 ****************************************************************************/

PttSerial::PttSerial(void)
  : dev(0), instream(""), cnt(0)
{
} /* PttSerial::PttSerial */


PttSerial::~PttSerial(void)
{
  dev->close();
  delete dev;
} /* PttSerial::~PttSerial */


bool PttSerial::initialize(Async::Config &cfg, const std::string name)
{

  string ptt_serial;
  if (!cfg.getValue(name, "PTT_PORT", ptt_serial))
  {
    cerr << "*** ERROR: Config variable " << name << "/PTT_PORT not set\n";
    return false;
  }
  dev = new Serial(ptt_serial);
  dev->charactersReceived.connect(
   	  mem_fun(*this, &PttSerial::onCharactersReceived));

  if (!dev->open(true))
  {
     cerr << "*** ERROR: Open serial port " << name << "/PTT_SERIAL="
         << ptt_serial << " failed.\n";
    return false;
  }

  // getting parameters for the serial port
  // PTT_PORT_PARAMETERS=19200,8,N,2
  std::string ptt_serial_parameters;
  int baud = 19200;
  int bits = 8;
  int stopbits = 2;
  Serial::Parity parity = Serial::PARITY_NONE;

  if (cfg.getValue(name, "PTT_PORT_PARAMETERS", ptt_serial_parameters))
  {
    std::vector<string> params;
    int cnt = SvxLink::splitStr(params, ptt_serial_parameters, ",");
    if (cnt != 4)
    {
      cout << "*** ERROR: Parameter " << name << "/PTT_PORT_PARAMETERS must have"
           << " this structure =19200,8,N,2\n";
      return false;
    }
    baud = atoi(params[0].c_str());
    bits = atoi(params[1].c_str());
    stopbits = atoi(params[3].c_str());

    if (params[2] == "N")
    {
      parity = Serial::PARITY_NONE;
    }
    else if (params[2] == "O")
    {
      parity = Serial::PARITY_ODD;
    }
    else if (params[2] == "E")
    {
      parity = Serial::PARITY_EVEN;
    }
    else
    {
      parity = Serial::PARITY_NONE;
    }
  }

  if (!dev->setParams(baud, parity, bits, stopbits, Serial::FLOW_NONE))
  {
    cout << "*** ERROR: Setting parameters on " << name << "/PTT_SERIAL "
         << "failed ("<< ptt_serial_parameters << ")\n";
    return false;
  }

  ptt_on[0] = 'T';
  ptt_off[0] = 'R';

  string ptt_serial_on;
  if (cfg.getValue(name, "PTT_ON", ptt_serial_on))
  {
    vector<string> cmds;
    ptt_on_cnt = SvxLink::splitStr(cmds, ptt_serial_on, " ");
    int a = 0;
    for (auto it : cmds)
    {
      unsigned int tt;
      sscanf(it.data(),"%02x", &tt);
      ptt_on[a++] = tt;
    }
  }

  string ptt_serial_off;
  if (cfg.getValue(name, "PTT_OFF", ptt_serial_off))
  {
    vector<string> cmds;
    ptt_off_cnt = SvxLink::splitStr(cmds, ptt_serial_off, " ");
    int a = 0;
    for (auto it : cmds)
    {
      unsigned int tt;
      sscanf(it.data(),"%02x", &tt);
      ptt_off[a++] = tt;
    }
  }

  // this must be send after switching Tx on/off, otherwise no audio
  // is forwart to the tx
  string ptt_serial_chg;
  if (cfg.getValue(name, "PTT_CHG", ptt_serial_chg))
  {
    vector<string> cmds;
    ptt_chg_cnt = SvxLink::splitStr(cmds, ptt_serial_chg, " ");
    int a = 0;
    for (auto it : cmds)
    {
      unsigned int tt;
      sscanf(it.data(),"%02x", &tt);
      ptt_chg[a++] = tt;
    }
  }

  return true;
} /* PttSerial::initialize */


/*
 * This functions sends a character over the serial-device (standard version):
 * T  to direct the controller to enable the TX
 * R  to direct the controller to disable the TX
 */
bool PttSerial::setTxOn(bool tx_on)
{
  tx_changed = true;
  char* cmd(tx_on ? ptt_on : ptt_off);
  int count(tx_on ? ptt_on_cnt : ptt_off_cnt);
  return sendMsg(cmd, count);
} /* PttSerial::setTxOn */



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

bool PttSerial::sendMsg(char *buf, int count)
{
  return (dev->write((const char*)buf, count) == count);
} /* PttSerial::sendMsg */


void PttSerial::onCharactersReceived(char *buf, int count)
{

  memcpy(instream+cnt, buf, count);
  cnt += count;
  int mkr = 0;

  for (int a=0; a<cnt; a++)
  {
    if (instream[a] == 0xfd)  // ßxfd marks the end of a command
    {
      char* t;
      int b;

      // ifnore "0xfe 0xfe" at the beginning and "0xfd" at the end
      for (b=mkr+2; b<a; b++)
      {
//      printf("%02x ", (unsigned int)instream[b]);
        t += instream[b];
      }
//      cout << endl;

      handleAnswer(t, b);
      mkr = a+1;
    }
  }

  if (mkr != 0)
  {
    memmove(instream, instream+mkr, cnt-mkr);
    cnt -= mkr;
  }

  if (tx_changed)
  {
    char* cmd = ptt_chg;
    cout << "SEND TXCHG: " << (tx_changed ? "TRUE":"FALSE") << endl;
    if (!sendMsg(cmd, ptt_chg_cnt))
    {
      cout << "*** ERROR: Sending TXCHG-message to G90\n";
    }
    tx_changed = false;
  }
} /* PttSerial::onCharactersReceived */


void PttSerial::handleAnswer(char* buf, int count)
{
  // cout << "received " << count << " digits\n";
} /* PttSerial::handleAnswer */


/*
 * This file has not been truncated
 */

