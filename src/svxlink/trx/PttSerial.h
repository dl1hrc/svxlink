/**
@file	 PttSerial.h
@brief   A PTT hardware controller using a Serial to signal an external script
@author  Tobias Blomberg / SM0SVX & Adi Bier / DL1HRC
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

#ifndef PTT_SERIAL_INCLUDED
#define PTT_SERIAL_INCLUDED


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

#include <AsyncSerial.h>

/****************************************************************************
 *
 * Local Includes
 *
 ****************************************************************************/

#include "Ptt.h"


/****************************************************************************
 *
 * Forward declarations
 *
 ****************************************************************************/



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
 * Exported Global Variables
 *
 ****************************************************************************/



/****************************************************************************
 *
 * Class definitions
 *
 ****************************************************************************/

/**
@brief	A PTT hardware controller using a Serial to signal an external script
@author Tobias Blomberg / SM0SVX & Adi Bier / DL1HRC
@date   2024-10-01
*/
class PttSerial : public Ptt
{
  public:
    struct Factory : public PttFactory<PttSerial>
    {
      Factory(void) : PttFactory<PttSerial>("SERIAL") {}
    };

    /**
     * @brief 	Default constructor
     */
    PttSerial(void);

    /**
     * @brief 	Destructor
     */
    ~PttSerial(void);

    /**
     * @brief 	Initialize the PTT hardware
     * @param 	cfg An initialized config object
     * @param   name The name of the config section to read config from
     * @returns Returns \em true on success or else \em false
     */
    virtual bool initialize(Async::Config &cfg, const std::string name);

    /**
     * @brief 	Set the state of the PTT, TX on or off
     * @param 	tx_on Set to \em true to turn the transmitter on
     * @returns Returns \em true on success or else \em false
     */
    virtual bool setTxOn(bool tx_on);

  protected:

  private:
    Async::Serial *dev;

    char ptt_on[15];
    char ptt_off[15];
    char ptt_chg[15];
    int ptt_on_cnt;
    int ptt_off_cnt;
    int ptt_chg_cnt;
    bool tx_changed;
    char instream[100];
    int cnt;
    int mkr;

    PttSerial(const PttSerial&);
    PttSerial& operator=(const PttSerial&);

    void onCharactersReceived(char *buf, int count);
    bool sendMsg(char *buf, int count);
    void handleAnswer(char *buf, int count);

};  /* class PttSerial */


#endif /* PTT_SERIAL_INCLUDED */


/*
 * This file has not been truncated
 */
