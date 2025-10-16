/**
@file	 TetraLogic.cpp
@brief   Contains a Tetra logic SvxLink core implementation
@author  Tobias Blomberg / SM0SVX & Adi Bier / DL1HRC
@date	 2020-05-27

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

#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>
#include <stdio.h>
#include <iostream>
#include <regex.h>
#include <fstream>


/****************************************************************************
 *
 * Project Includes
 *
 ****************************************************************************/

#include <Rx.h>
#include <Tx.h>
#include <AsyncTimer.h>
#include <json/json.h>


/****************************************************************************
 *
 * Local Includes
 *
 ****************************************************************************/

#include "TetraLogic.h"
#include "TetraLib.h"
#include "common.h"
#include "EventHandler.h"
#include "Module.h"
#include "config.h"

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

#define OK 0
#define ERROR 1
#define CALL_BEGIN 3
#define GROUPCALL_END 4
#define REGISTRATION 5
#define SDS 6
#define TEXT_SDS 7
#define CNUMF 8
#define CALL_CONNECT 9
#define TRANSMISSION_END 10
#define CALL_RELEASED 11
#define LIP_SDS 12
#define REGISTER_TSI 13
#define STATE_SDS 14
#define OP_MODE 15
#define TRANSMISSION_GRANT 16
#define TX_DEMAND 17
#define TX_WAIT 18
#define TX_INTERRUPT 19
#define SIMPLE_LIP_SDS 20
#define COMPLEX_SDS 21
#define MS_CNUM 22
#define WAP_PROTOCOL 23
#define SIMPLE_TEXT_SDS 24
#define ACK_SDS 25
#define CMGS 26
#define CONCAT_SDS 27
#define CTGS 28
#define CTDGR 29
#define CLVL 30
#define OTAK 31
#define WAP_MESSAGE 32
#define LOCATION_SYSTEM_TSDU 33
#define RSSI 34
#define VENDOR 35
#define MODEL 36

#define DMO_OFF 7
#define DMO_ON 8

#define INVALID 254
#define TIMEOUT 255

#define LOGERROR 0
#define LOGWARN 1
#define LOGINFO 2
#define LOGDEBUG 3
#define LOGTRACE 4

#define MAX_TRIES 5

#define TETRA_LOGIC_VERSION "16102025"

/****************************************************************************
 *
 * Local class definitions
 *
 ****************************************************************************/

class TetraLogic::Call
{

  public:

  Call()
  {

  }

  ~Call()
  {

  }

  private:

};



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

extern "C" {
  Logic* construct(void) { return new TetraLogic; }
}


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


TetraLogic::TetraLogic(void)
  : mute_rx_on_tx(true), mute_tx_on_rx(true),
  rgr_sound_always(false), mcc(""), mnc(""), issi(""), gssi(1),
  port("/dev/ttyUSB0"), baudrate(115200), initstr("AT+CTOM=1;AT+CTSP=1,3,131"),
  pei(0), sds_pty(0), peistream(""), debug(LOGERROR), talkgroup_up(false),
  sds_when_dmo_on(false), sds_when_dmo_off(false),
  sds_when_proximity(false), peiComTimer(2000, Timer::TYPE_ONESHOT, false),
  peiActivityTimer(10000, Timer::TYPE_ONESHOT, true),
  peiBreakCommandTimer(3000, Timer::TYPE_ONESHOT, false),
  t_aprs_sym('E'), t_aprs_tab('/'),
  proximity_warning(3.1), time_between_sds(3600), own_lat(0.0),
  own_lon(0.0), endCmd(""), new_sds(false), inTransmission(false),
  cmgs_received(true), share_userinfo(true), current_cci(0), dmnc(0),
  dmcc(0), dissi(0), infosds(""), is_tx(false), last_sdsid(0), pei_pty_path(""),
  pei_pty(0), ai(-1), check_qos(0), qos_sds_to("0815"), qos_limit(-90),
  qosTimer(300000, Timer::TYPE_ONESHOT, false),
  userRegTimer(120000, Timer::TYPE_ONESHOT, false), min_rssi(100), max_rssi(100),
  reg_cell(0), reg_la(0), reg_mni(0), vendor(""), model(""), inactive_time(3600)
{
  peiComTimer.expired.connect(mem_fun(*this, &TetraLogic::onComTimeout));
  peiActivityTimer.expired.connect(mem_fun(*this,
                            &TetraLogic::onPeiActivityTimeout));
  peiBreakCommandTimer.expired.connect(mem_fun(*this,
                            &TetraLogic::onPeiBreakCommandTimeout));
  qosTimer.expired.connect(mem_fun(*this, &TetraLogic::onQosTimeout));
  userRegTimer.expired.connect(mem_fun(*this, &TetraLogic::userRegTimeout));
} /* TetraLogic::TetraLogic */


TetraLogic::~TetraLogic(void)
{
  peiComTimer = 0;
  peiActivityTimer = 0;
  peiBreakCommandTimer = 0;
  qosTimer = 0;
} /* TetraLogic::~TetraLogic */


bool TetraLogic::initialize(Async::Config& cfgobj, const std::string& logic_name)
{
  bool isok = true;
  if (!Logic::initialize(cfgobj, logic_name))
  {
    isok = false;
  }

   // get own position
  if (LocationInfo::has_instance())
  {
    own_lat = getDecimalDegree(LocationInfo::instance()->getCoordinate(true));
    own_lon = getDecimalDegree(LocationInfo::instance()->getCoordinate(false));
  }
  cfg().getValue(name(), "MUTE_RX_ON_TX", mute_rx_on_tx);
  cfg().getValue(name(), "MUTE_TX_ON_RX", mute_tx_on_rx);
  cfg().getValue(name(), "RGR_SOUND_ALWAYS", rgr_sound_always);

  string value;
  if (!cfg().getValue(name(), "ISSI", issi))
  {
     log(LOGERROR, "*** ERROR: Missing parameter " + name() + "/ISSI");
     isok = false;
  }

  cfg().getValue(name(), "GSSI", gssi);

  if (!cfg().getValue(name(), "MCC", mcc))
  {
     log(LOGERROR, "*** ERROR: Missing parameter " + name() + "/MCC");
     isok = false;
  }
  if (atoi(mcc.c_str()) > 901)
  {
     log(LOGERROR, "*** ERROR: Country code (MCC) must be 901 or less");
     isok = false;
  }
  if (mcc.length() < 4)
  {
    value = "0000";
    value += mcc;
    mcc = value.substr(value.length()-4,4);
  }
  dmcc = atoi(mcc.c_str());

  if (!cfg().getValue(name(), "APRSPATH", aprspath))
  {
    aprspath = "APRS,qAR,";
    aprspath += callsign();
    aprspath += "-10:";
  }
  if (!cfg().getValue(name(), "MNC", mnc))
  {
    log(LOGERROR, "*** ERROR: Missing parameter " + name() + "/MNC");
    isok = false;
  }
  if (atoi(mnc.c_str()) > 16383)
  {
    log(LOGERROR, "*** ERROR: Network code (MNC) must be 16383 or less");
    isok = false;
  }
  if (mnc.length() < 5)
  {
    value = "00000";
    value += mnc;
    mnc = value.substr(value.length()-5,5);
  }
  dmnc = atoi(mnc.c_str());

  // Welcome message to new users
  cfg().getValue(name(), "INFO_SDS", infosds);
  cfg().getValue(name(), "DEBUG", debug);

  if (!cfg().getValue(name(), "PORT", port))
  {
    log(LOGWARN, "Missing parameter " + name() + "/PORT");
    isok = false;
  }

  if (!cfg().getValue(name(), "BAUD", baudrate))
  {
    log(LOGWARN, "Missing parameter " + name()
        + "/BAUD, guess " + to_string(baudrate));
  }

  if (cfg().getValue(name(), "DEFAULT_APRS_ICON", value))
  {
    if (value.length() != 2)
    {
      isok = false;
      stringstream ss;
      ss << "*** ERROR: " << name() << "/DEFAULT_APRS_ICON "
           << "must have 2 characters, e.g. '/e' or if the backslash or "
           << "a comma is used it has to be encoded with an additional "
           << "'\', e.g. " << "DEFAULT_APRS_ICON=\\r";
      log(LOGERROR, ss.str());
    }
    else
    {
      t_aprs_sym = value[0];
      t_aprs_tab = value[1];
    }
  }

  // the pty path: inject messages to send by Sds
  string sds_pty_path;
  cfg().getValue(name(), "SDS_PTY", sds_pty_path);
  if (!sds_pty_path.empty())
  {
    sds_pty = new Pty(sds_pty_path);
    if (!sds_pty->open())
    {
      log(LOGERROR, "*** ERROR: Could not open Sds PTY " +
        sds_pty_path + " as specified in configuration variable " +
        name() + "/" + "SDS_PTY");
      isok = false;
    }
    sds_pty->dataReceived.connect(
        mem_fun(*this, &TetraLogic::sdsPtyReceived));
  }

  list<string>::iterator slit;

  string user_section;
  if (cfg().getValue(name(), "TETRA_USERS", user_section))
  {
    cout
      << "***************************************************************\n"
      << "* WARNING: The parameter TETRA_USERS is outdated and will be  *\n"
      << "* removed soon. Use TETRA_USER_INFOFILE=tetra_users.json in-  *\n"
      << "* stead and transfer your tetra user data into the json file. *\n"
      << "* You will find an example of tetra_users.json in             *\n"
      << "* /src/svxlink/svxlink.d directory                            *\n"
      << "***************************************************************\n";
    list<string> user_list = cfg().listSection(user_section);
    User m_user;

    for (slit=user_list.begin(); slit!=user_list.end(); slit++)
    {
      cfg().getValue(user_section, *slit, value);
      if ((*slit).length() != 17)
      {
        cout << "*** ERROR: Wrong length of TSI in TETRA_USERS definition, "
             << "should have 17 digits (MCC[4] MNC[5] ISSI[8]), e.g. "
             << "09011638312345678" << endl;
        isok = false;
      }
      else
      {
        m_user.issi = *slit;
        m_user.call = getNextStr(value);
        m_user.name = getNextStr(value);
        std::string m_aprs = getNextStr(value);
        if (m_aprs.length() != 2)
        {
          cout << "*** ERROR: Check Aprs icon definition for " << m_user.call
               << " in section " << user_section
               << ". It must have exactly 2 characters, e.g.: 'e\'" << endl;
          isok = false;
        }
        else
        {
          m_user.aprs_sym = m_aprs[0];
          m_user.aprs_tab = m_aprs[1];
        }
        m_user.comment = getNextStr(value); // comment for each user
        struct tm mtime = {0}; // set default date/time 31.12.1899
        m_user.last_activity = mktime(&mtime);
        m_user.sent_last_sds = mktime(&mtime);
        userdata[*slit] = m_user;
      }
    }
  }

  std::string user_info_file;
  if (cfg().getValue(name(), "TETRA_USER_INFOFILE", user_info_file))
  {
    std::ifstream user_info_is(user_info_file.c_str(), std::ios::in);
    if (user_info_is.good())
    {
      try
      {
        if (!(user_info_is >> m_user_info))
        {
          std::cerr << "*** ERROR: Failure while reading user information file "
                       "\"" << user_info_file << "\""
                    << std::endl;
          isok = false;
        }
      }
      catch (const Json::Exception& e)
      {
        std::cerr << "*** ERROR: Failure while reading user information "
                     "file \"" << user_info_file << "\": "
                  << e.what()
                  << std::endl;
        isok = false;
      }
    }
    else
    {
      std::cerr << "*** ERROR: Could not open user information file "
                   "\"" << user_info_file << "\""
                << std::endl;
      isok = false;
    }

    User m_user;
    for (Json::Value::ArrayIndex i = 0; i < m_user_info.size(); i++)
    {
      Json::Value& t_userdata = m_user_info[i];
      m_user.issi = t_userdata.get("tsi", "").asString();
      if (m_user.issi.length() != 17)
      {
        cout << "*** ERROR: The TSI must have a length of 17 digits.\n"
          << "\" Check dataset " << i + 1 << " in \"" << user_info_file
          << "\"" << endl;
        isok = false;
      }
      m_user.name = t_userdata.get("name","").asString();
      m_user.mode = t_userdata.get("mode","TETRA").asString();
      m_user.call = t_userdata.get("call","").asString();
      m_user.idtype = t_userdata.get("idtype","tsi").asString();
      m_user.location = t_userdata.get("location","").asString();
      if (t_userdata.get("symbol","").asString().length() != 2)
      {
        cout << "*** ERROR: Aprs symbol in \"" << user_info_file
           << "\" dataset " << i + 1 << " is not correct, must have 2 digits!"
           << endl;
        isok = false;
      }
      else
      {
        m_user.aprs_sym = t_userdata.get("symbol","").asString()[0];
        m_user.aprs_tab = t_userdata.get("symbol","").asString()[1];
      }
      m_user.comment = t_userdata.get("comment","").asString();
      struct tm mtime = {0}; // set default date/time 31.12.1899
      m_user.last_activity = mktime(&mtime);
      m_user.sent_last_sds = mktime(&mtime);
      userdata[m_user.issi] = m_user;
      log(LOGINFO, "Tsi=" + m_user.issi + ", call=" + m_user.call
             + ", name=" + m_user.name + ", location=" + m_user.location
             + ", comment=" + m_user.comment);
    }
  }

    // the init-pei file where init AT commands are defined
  std::string pei_init_file;
  if (cfg().getValue(name(), "PEI_INIT_FILE", pei_init_file))
  {
    std::ifstream pei_init_is(pei_init_file.c_str(), std::ios::in);
    if (pei_init_is.good())
    {
      try
      {
        if (!(pei_init_is >> m_pei_init))
        {
          std::cerr << "*** ERROR: Failure while reading pei-init information file "
                       "\"" << pei_init_file << "\""
                    << std::endl;
          isok = false;
        }
      }
      catch (const Json::Exception& e)
      {
        std::cerr << "*** ERROR: Failure while reading pei-init information "
                     "file \"" << pei_init_file << "\": "
                  << e.what()
                  << std::endl;
        isok = false;
      }
    }
    else
    {
      std::cerr << "*** ERROR: Could not open pei-init information file "
                   "\"" << pei_init_file << "\""
                << std::endl;
      isok = false;
    }

     // valid: TMO, DMO-MS, DMO-REPEATER, GATEWAY
    std::string tetra_mode = "DMO-MS";
    cfg().getValue(name(), "TETRA_MODE", tetra_mode);

    for (Json::Value::ArrayIndex i = 0; i < m_pei_init.size(); i++)
    {
      Json::Value& t_peiinit = m_pei_init[i];
      if (tetra_mode == t_peiinit.get("mode","").asString())
      {
        Json::Value& t_a = t_peiinit["commands"];
        log(LOGDEBUG,
        "+++ Reading AT-commands to initialze PEI-device. Reading from file" +
        pei_init_file + "\"");
        for (Json::Value::ArrayIndex j = 0; j < t_a.size(); j++)
        {
          m_cmds.push_back(t_a[j].asString());
          log(LOGDEBUG, "    " + t_a[j].asString());
        }
      }
    }
  }
    // initializes the Pei device by parameter INIT_PEI in svxlink.conf
  else if (cfg().getValue(name(), "INIT_PEI", initstr))
  {
    SvxLink::splitStr(initcmds, initstr, ";");
    m_cmds = initcmds;
    cout << "+++ WARNING: INIT_PEI is outdated and is being ignored " <<
      "in further versions of tetra-contrib. Please change your " <<
      "configuration and use the pei-init.json to define AT " <<
      "initializing commands. Please also read the manual page." << endl;
    log(LOGDEBUG,
      "+++ Reading AT commands by using the parameter svxlink.conf/INIT_PEI=");
  }
  else
  {
    cout << "+++ WARNING: No PEI initializing sequence defined, you " <<
      "should configure the parameter PEI_INIT_FILE in your TetraLogic.conf " <<
      "in svxlink.d directory. Please also read the manual page." << endl;
  }

  // define sds messages send to user when received Sds's from him due to
  // state changes
  std::string sds_useractivity;
  if (cfg().getValue(name(), "SDS_ON_USERACTIVITY", sds_useractivity))
  {
    list<string> activity_list = cfg().listSection(sds_useractivity);
    for (slit=activity_list.begin(); slit!=activity_list.end(); slit++)
    {
      cfg().getValue(sds_useractivity, *slit, value);
      if (value.length() > 100)
      {
        cout << "+++ WARNING: Message to long (>100 digits) at " << name()
             << "/" << sds_useractivity << ": " << (*slit)
             << ". Cutting message.";
        sds_on_activity[atoi((*slit).c_str())] = value.substr(0,100);
      }
      else
      {
        sds_on_activity[atoi((*slit).c_str())] = value;
      }
    }
  }

  // a section that combine SDS and a command:
  // 32768=1234
  std::string sds_to_cmd;
  unsigned int isds;
  if (cfg().getValue(name(), "SDS_TO_COMMAND", sds_to_cmd))
  {
    list<string> sds2cmd_list = cfg().listSection(sds_to_cmd);
    for (slit=sds2cmd_list.begin(); slit!=sds2cmd_list.end(); slit++)
    {
      cfg().getValue(sds_to_cmd, *slit, value);
      isds = static_cast<unsigned int>(std::stoul(*slit));
      if (isds < 32768 || isds > 65535)
      {
        cout << "*** ERROR: Sds decimal value in section " << name()
             << "/SDS_TO_COMMAND is not valid (" << isds
             << "), must be between 32768 and 65535" << endl;
      }
      else
      {
        if (debug >= LOGINFO)
        {
          cout << to_string(isds) << "=" << value << endl;
        }
        sds_to_command[isds] = value;
      }
    }
  }

  // define if Sds's send to all other users if the state of one user is
  // changed at the moment only: DMO_ON, DMO_OFF, PROXIMITY
  std::string sds_othersactivity;
  if (cfg().getValue(name(), "SDS_TO_OTHERS_ON_ACTIVITY", sds_othersactivity))
  {
    string::iterator comma;
    string::iterator begin = sds_othersactivity.begin();
    do
    {
      comma = find(begin, sds_othersactivity.end(), ',');
      string item;
      if (comma == sds_othersactivity.end())
      {
        item = string(begin, sds_othersactivity.end());
      }
      else
      {
        item = string(begin, comma);
        begin = comma + 1;
      }
      if (item == "DMO_ON")
      {
        sds_when_dmo_on = true;
      }
      else if (item == "DMO_OFF")
      {
        sds_when_dmo_off = true;
      }
      else if (item == "PROXIMITY")
      {
        sds_when_proximity = true;
      }
    } while (comma != sds_othersactivity.end());
  }

  // read info of tetra state to receive SDS's
  std::string status_section;
  if (cfg().getValue(name(), "TETRA_STATUS", status_section))
  {
    list<string> state_list = cfg().listSection(status_section);
    for (slit=state_list.begin(); slit!=state_list.end(); slit++)
    {
      cfg().getValue(status_section, *slit, value);
      isds = static_cast<unsigned int>(std::stoul(*slit));
      if(isds < 32768 || isds > 65536)
      {
        stringstream ss;
        ss << "*** ERROR: Sds decimal value in section " << name() <<
          "/TETRA_STATUS is not valid (" << isds <<
          "), must be between 32768 and 65535";
        log(LOGERROR, ss.str());
      }
      else
      {
        log(LOGINFO, to_string(isds) + "=" + value);
        state_sds[isds] = value;
      }
    }
  }

  if (cfg().getValue(name(), "PROXIMITY_WARNING", value))
  {
    proximity_warning = atof(value.c_str());
  }

  if (cfg().getValue(name(), "TIME_BETWEEN_SDS", value))
  {
    time_between_sds = atoi(value.c_str());
  }

  if (cfg().getValue(name(), "INACTIVE_AFTER", value))
  {
    inactive_time = atoi(value.c_str());
    if (inactive_time < 100 || inactive_time > 14400)
    {
      inactive_time = 3600;
    }
  }

  cfg().getValue(name(), "END_CMD", endCmd);

  std::string dapnet_server;
  if (cfg().getValue(name(), "DAPNET_SERVER", dapnet_server))
  {
    dapnetclient = new DapNetClient(cfg(), name());
    dapnetclient->dapnetMessageReceived.connect(mem_fun(*this,
                    &TetraLogic::onDapnetMessage));
    dapnetclient->dapnetLogmessage.connect(mem_fun(*this,
                    &TetraLogic::onDapnetLogmessage));
    if (!dapnetclient->initialize())
    {
      log(LOGERROR, "*** ERROR: initializing DAPNET client");
      isok = false;
    }
  }

  cfg().getValue(name(),"SHARE_USERINFO", share_userinfo);

    // PEI pty path: inject messages to send to PEI directly
  cfg().getValue(name(), "PEI_PTY", pei_pty_path);
  if (!pei_pty_path.empty())
  {
    pei_pty = new Pty(pei_pty_path);
    if (!pei_pty->open())
    {
      log(LOGERROR, "*** ERROR: Could not open Pei PTY " +
        pei_pty_path + " as specified in configuration variable " +
        name() + "/" + "PEI_PTY");
      isok = false;
    }
    pei_pty->dataReceived.connect(
        mem_fun(*this, &TetraLogic::peiPtyReceived));
  }

  if (cfg().getValue(name(),"CHECK_QOS", check_qos))
  {
    cfg().getValue(name(), "QOS_EMAIL_TO", qos_email_to);
    cfg().getValue(name(), "QOS_SDS_TO", qos_sds_to);
    cfg().getValue(name(), "QOS_LIMIT", qos_limit);
    if (check_qos < 30 || check_qos > 6000) check_qos = 30;
    qosTimer.setTimeout(check_qos * 1000);
    qosTimer.reset();
    qosTimer.setEnable(true);
    log(LOGDEBUG, "QOS enabled");
  }

  // start the user registration timer, after expiration
  // it checks the registration state of the registered users
  userRegTimer.setEnable(true);

  // hadle the Pei serial port
  pei = new Serial(port);
  if (!pei->open())
  {
    log(LOGERROR, "*** ERROR: Opening serial port " + name() + "/PORT="
         + port);
    isok = false;
  }
  pei->setParams(baudrate, Serial::PARITY_NONE, 8, 1, Serial::FLOW_NONE);
  pei->charactersReceived.connect(
      	  mem_fun(*this, &TetraLogic::onCharactersReceived));

  sendPei("\r\n");

   // receive interlogic messages
  publishStateEvent.connect(
          mem_fun(*this, &TetraLogic::onPublishStateEvent));

  peirequest = AT_CMD_WAIT;
  initPei();

  rxValveSetOpen(true);
  setTxCtrlMode(Tx::TX_AUTO);

  /** New Tetra event methods **/
  Logic::logic_event_handler()->sendSds.connect(sigc::mem_fun(*this,
                                     &TetraLogic::sendSds));
  Logic::logic_event_handler()->setupCall.connect(sigc::mem_fun(*this,
                                     &TetraLogic::setupCall));
  Logic::logic_event_handler()->dummy.connect(sigc::mem_fun(*this,
                                     &TetraLogic::dummy));

  processEvent("startup");

  cout << "TetraLogic version "<< string(TETRA_LOGIC_VERSION) << " started."
       << endl;
  log(LOGINFO, ">>> Started SvxLink with special TetraLogic extension (v" +
                   string(TETRA_LOGIC_VERSION) + ")");
  log(LOGINFO, ">>> No guarantee! Please send a bug report to");
  log(LOGINFO, ">>> Adi/DL1HRC <dl1hrc@gmx.de> or use the groups.io mailing list");

  // std::string mysds;
  // createSDS(mysds, "2620055", "Das ist ein Test vom lieben Adi.");
  // cout << "SDS:" << mysds;

  // Test/Debug entries for bug detection, normally comment out
  // std::string sds = "0A0BA7D5B95BC50AFFE160";
  // std::string sds = "0A00893E12472C51026810"; // DL1HRC
  // std::string sds = "0A112853A9FF4D4FFFE810";
  // std::string sds = "0A06D0103EC61871000810"; // VK
  // LipInfo li;
  // handleLipSds(sds, li);
  /* cout << "Lipinfo: " << sds << endl;
    cout << "Result, lat=" << dec2nmea_lat(li.latitude) << ", lon="
         << dec2nmea_lon(li.longitude) << ", pos error=" << li.positionerror
         << ", horizontalvel=" << li.horizontalvelocity << ", directionoftravel="
         << li.directionoftravel << ", reasonforsending="
         << li.reasonforsending << endl;
    cout << "Latitude=" << li.latitude << ", longitude=" << li.longitude << endl;
  */
  return isok;

} /* TetraLogic::initialize */


void TetraLogic::remoteCmdReceived(Logic* src_logic, const std::string& cmd)
{
  log(LOGTRACE, "TetraLogic::remoteCmdReceived: "
           + src_logic->name() + " -> " + cmd);
} /* TetraLogic::remoteCmdReceived */


/****************************************************************************
 *
 * Protected member functions
 *
 ****************************************************************************/

 void TetraLogic::allMsgsWritten(void)
{
  Logic::allMsgsWritten();
  if (!talkgroup_up)
  {
    setTxCtrlMode(Tx::TX_AUTO);
  }
} /* TetraLogic::allMsgsWritten */


void TetraLogic::audioStreamStateChange(bool is_active, bool is_idle)
{
  Logic::audioStreamStateChange(is_active, is_idle);
} /* TetraLogic::audioStreamStateChange */


void TetraLogic::transmitterStateChange(bool is_transmitting)
{

  std::string cmd;
  is_tx = is_transmitting;

  if (is_transmitting)
  {
    if (!talkgroup_up)
    {
      log(LOGTRACE, "TetraLogic::transmitterStateChange: " + gssi);
      initGroupCall(gssi);
      talkgroup_up = true;
    }
    else
    {
      cmd = "AT+CTXD=";
      cmd += std::to_string(current_cci);
      cmd += ",1";
      sendPei(cmd);
    }
  }
  else
  {
    cmd = "AT+CUTXC=";
    cmd += std::to_string(current_cci);
    sendPei(cmd);
  }

  if (mute_rx_on_tx)
  {
   // rx().setMuteState(is_transmitting ? Rx::MUTE_ALL : Rx::MUTE_NONE);
  }

  Logic::transmitterStateChange(is_transmitting);
} /* TetraLogic::transmitterStateChange */


void TetraLogic::squelchOpen(bool is_open)
{
  // FIXME: A squelch open should not be possible to receive while
  // transmitting unless mute_rx_on_tx is false, in which case it
  // should be allowed. Commenting out the statements below.
  std::string s = (is_open ? "true" : "false");
  log(LOGTRACE, "TetraLogic::squelchOpen: squelchopen=" + s);

  if (tx().isTransmitting())
  {
    log(LOGTRACE, "TetraLogic::squelchOpen: tx().isTransmitting()=true");
    return;
  }

  // preparing dynamical TG request
  // Logic::setReceivedTg(9);

  log(LOGTRACE, "TetraLogic::squelchOpen: rx().setSql(" + s + ")");
  rx().setSql(is_open);
  log(LOGTRACE, "TetraLogic::squelchOpen: Logic::squelchOpen(" + s + ")");
  Logic::squelchOpen(is_open);
} /* TetraLogic::squelchOpen */


/****************************************************************************
 *
 * Private member functions
 *
 ****************************************************************************/


 /*
  Initialize the Pei device, here some commends that being used
  to (re)direct the answers to the Pei port. See EN 300 392-5
  V2.2.0 manual, page 62 for further info

  TETRA Service Profile +CTSP:
  +CTSP=<service profile>, <service layer1>, [<service layer2>],
        [<AI mode>], [<link identifier>]

  AT+CTOM=1           set MRT into DMO-MS mode (0-TMO, 6-DMO-Repeater)
  AT+CTSP=1,3,131     Short Data Service type 4 with Transport Layer (SDS-TL)
                      service
                      131 - GPS
  AT+CTSP=1,3,130     130 - Text Messaging
  AT+CTSP=1,2,20      Short Data Service (SDS)
                      20 - Status
  AT+CTSP=2,0,0       0 - Voice
  AT+CTSP=1,3,24      24 - SDS type 4, PID values 0 to 127
  AT+CTSP=1,3,25      25 - SDS type 4, PID values 128 to 255
  AT+CTSP=1,3,3       3 - Simple GPS
  AT+CTSP=1,3,10      10 - Location information protocol
  AT+CTSP=1,1,11      11 - Group Management

  TETRA service definition for Circuit Mode services +CTSDC
  +CTSDC=<AI service>, <called party identity type>, [<area>], [<hook>],
         [<simplex>], [<end to end encryption>],[<comms type>],
         [<slots/codec>], [<RqTx>], [<priority>], [<CLIR control>]
  AT+CTSDC=0,0,0,1,1,0,1,1,0,0
 */
void TetraLogic::initPei(void)
{
  stringstream ss;
  std::string cmd;

  if (peirequest == AT_CMD_WAIT)
  {
    peiBreakCommandTimer.reset();
    peiBreakCommandTimer.setEnable(true);
  }
  if (!m_cmds.empty())
  {
    cmd = *(m_cmds.begin());
    sendPei(cmd);
    m_cmds.erase(m_cmds.begin());
  }
  else if (peirequest == INIT)
  {
    cmd = "AT+CNUMF?"; // get the MCC,MNC,ISSI from MS
    sendPei(cmd);
    ss << "pei_init_finished";
    processEvent(ss.str());
    sendUserInfo(); // send userinfo to reflector
    if (vendor.length() > 1) sendSystemInfo(); // send systeminfo to reflector
    peirequest = INIT_COMPLETE;
  }
} /* TetraLogic::initPei */


void TetraLogic::sendUserInfo(void)
{

  // read infos of tetra users configured in svxlink.conf
  Json::Value event(Json::arrayValue);
  int a = 0;

  for (std::map<std::string, User>::iterator iu = userdata.begin();
       iu!=userdata.end(); iu++)
  {
    Json::Value t_userinfo(Json::objectValue);
    t_userinfo["id"] = iu->second.issi;
    t_userinfo["call"] = iu->second.call;
    t_userinfo["idtype"] = iu->second.idtype;
    t_userinfo["mode"] = iu->second.mode;
    t_userinfo["name"] = iu->second.name;
    t_userinfo["tab"] = iu->second.aprs_tab;
    t_userinfo["sym"] = iu->second.aprs_sym;
    t_userinfo["comment"] = iu->second.comment;
    t_userinfo["location"] = iu->second.location;
    t_userinfo["last_activity"] = 0;
    t_userinfo["registered"] = iu->second.registered;
    t_userinfo["message"] = "DvUsers:info";
    event.append(t_userinfo);
    if (a++ > 5)
    {
      a = 0;
      publishInfo("DvUsers:info", event);
      event.clear();
    }
  }
  publishInfo("DvUsers:info", event);
} /* TetraLogic::sendUserInfo */


void TetraLogic::onCharactersReceived(char *buf, int count)
{
  peiComTimer.setEnable(false);
  peiActivityTimer.reset();
  size_t found;

  peistream += buf;

  /*
  The asynchronous handling of incoming PEI commands is not easy due
  to the unpredictability of the reception of characters from the serial port.
  We have to analyze the incoming characters until we find the first
  \r\n-combination. Afterwards we are looking for a second occurrence,
  if one occurs, then we have an entire PEI command. The rest of the
  data is then left untouched.
  If we find a \r\n-combination after the second one, then it is most
  likely a SDS as an unsolicited answer just following the e.g.
  +CTSDSR:xxx message.
  */

  while ((found = peistream.find("\r\n")) != string::npos)
  {
    if (found != 0)
    {
      handlePeiAnswer(peistream.substr(0, found));
    }
    peistream.erase(0, found+2);
  }

} /* TetraLogic::onCharactersReceived */


void TetraLogic::handlePeiAnswer(std::string m_message)
{

  log(LOGDEBUG, "From PEI:" + m_message);

  int response = handleMessage(m_message);

  log(LOGTRACE, "TetraLogic::handlePeiAnswer: response=" + to_string(response));

  switch (response)
  {
    case OK:
      peistate = OK;
      if (new_sds) checkSds();
      break;

    case ERROR:
      peistate = ERROR;
      if (m_message.length()>11)
      {
        log(LOGINFO, getPeiError(atoi(m_message.erase(0,11).c_str())) );
      }
      break;

    case CNUMF:
      handleCnumf(m_message);
      break;

    case CALL_BEGIN:
      handleCallBegin(m_message);
      break;

    case TRANSMISSION_END:
      handleTransmissionEnd(m_message);
      break;

    case CALL_RELEASED:
      handleCallReleased(m_message);
      break;

    case SDS:
      handleSds(m_message);
      break;

    case ACK_SDS:
      break;

    case TEXT_SDS:
      handleSdsMsg(m_message);
      break;

    case SIMPLE_TEXT_SDS:
    case STATE_SDS:
      handleSdsMsg(m_message);
      break;

    case COMPLEX_SDS:
    case CONCAT_SDS:
    case LIP_SDS:
      handleSdsMsg(m_message);
      break;

    case CMGS:
      // +CMGS: <SDS Instance>[, <SDS status> [, <message reference>]]
      // sds state send be MS
      handleCmgs(m_message);
      break;

    case TX_DEMAND:
      break;

    case TRANSMISSION_GRANT:
      handleTxGrant(m_message);
      break;

    case CALL_CONNECT:
      current_cci = handleCci(m_message);
      break;

    case OP_MODE:
      ai = getAiMode(m_message);
      getRssi();
      break;

    case CTGS:
      handleCtgs(m_message);
      break;

    case CTDGR:
      log(LOGINFO, handleCtdgr(m_message));
      break;

    case CLVL:
      handleClvl(m_message);
      break;

    case RSSI:
      handleRssi(m_message);
      break;

    case REGISTRATION:
      handleCreg(m_message);
      break;

    case VENDOR:
      handleVendor(m_message);
      break;

    case MODEL:
      handleModel(m_message);
      break;

    case INVALID:
      log(LOGWARN, "+++ Pei answer not known, ignoring ;)");

    default:
      break;
  }

  if (peirequest == INIT && (response == OK || response == ERROR))
  {
    initPei();
  }

} /* TetraLogic::handlePeiAnswer */


void TetraLogic::initGroupCall(int gc_gssi)
{
  log(LOGTRACE, "TetraLogic::initGroupCall: " + to_string(gc_gssi));
  inTransmission = true;
  std::string cmd = "AT+CTSDC=0,0,0,1,1,0,1,1,0,0,0";
  sendPei(cmd);

  cmd = "ATD";
  cmd += to_string(gc_gssi);
  sendPei(cmd);

  stringstream ss;
  ss << "init_group_call " << to_string(gc_gssi);
  processEvent(ss.str());
} /* TetraLogic::initGroupCall */



/*
TETRA Incoming Call Notification +CTICN

+CTICN: <CC instance >, <call status>, <AI service>,
[<calling party identity type>], [<calling party identity>],
[<hook>], [<simplex>], [<end to end encryption>],
[<comms type>], [<slots/codec>], [<called party identity type>],
[<called party identity>], [<priority level>]

 Example:        MCC| MNC| ISSI  |             MCC| MNC|  GSSI |
 +CTICN: 1,0,0,5,09011638300023404,1,1,0,1,1,5,09011638300000001,0
 OR               ISSI             GSSI
 +CTICN: 1,0,0,5,23404,1,1,0,1,1,5,1000,0
*/
void TetraLogic::handleCallBegin(std::string message)
{
  log(LOGTRACE,"TetraLogic::handleCallBegin: " + message);

  //                   +CTICN:   1,    0,    0,    4,    1002,       1,     1,     0,   1,    1,   0,    1000,       1
  std::string reg = "\\+CTICN: [0-9]{1,3},[0-9],[0-9],[0-9],[0-9]{1,17},[0-9],[0-9],[0-9],[0-9],[0-9],[0-9],[0-9]{1,17},[0-9]";

  if (!rmatch(message, reg))
  {
    log(LOGWARN, "*** Wrong +CTICN response (wrong format)");
    return;
  }
  squelchOpen(true);  // open the Squelch
  stringstream tt;

  Callinfo t_ci;
  stringstream ss;
  message.erase(0,8);
  std::string h = message;

  // split the message received from the Pei into single parameters
  // for further use, not all of them are interesting
  t_ci.instance = getNextVal(h);
  t_ci.callstatus = getNextVal(h);
  t_ci.aistatus = getNextVal(h);
  t_ci.origin_cpit = getNextVal(h);

  // the original TSI
  std::string o_tsi = getNextStr(h);

  if (o_tsi.length() < 9)
  {
    t_ci.o_issi = atoi(o_tsi.c_str());
    string t = mcc;
    t += mnc;
    t += getISSI(o_tsi);
    o_tsi = t;
    t_ci.o_mnc = dmnc;
    t_ci.o_mcc = dmcc;
  }
  else
  {
    splitTsi(o_tsi, t_ci.o_mcc, t_ci.o_mnc, t_ci.o_issi);
  }

  // set the correct length of tsi to 17
  if (o_tsi.length() != 17)
  {
    tt << setfill('0') << setw(4) << t_ci.o_mcc << setfill('0') << setw(5) << t_ci.o_mnc << setfill('0') << setw(8) << t_ci.o_issi;
    o_tsi = tt.str();
  }

  t_ci.hook = getNextVal(h);
  t_ci.simplex = getNextVal(h);
  t_ci.e2eencryption = getNextVal(h);
  t_ci.commstype = getNextVal(h);
  t_ci.codec = getNextVal(h);
  t_ci.dest_cpit = getNextVal(h);

  std::string d_tsi = getNextStr(h);

  if (d_tsi.length() < 9)
  {
    t_ci.d_issi = atoi(d_tsi.c_str());
    string t = mcc;
    t += mnc;
    t += getISSI(d_tsi);
    d_tsi = t;
    t_ci.d_mnc = dmnc;
    t_ci.d_mcc = dmcc;
  }
  else
  {
    splitTsi(d_tsi, t_ci.d_mcc, t_ci.d_mnc, t_ci.d_issi);
  }

  // set the correct length of tsi to 17
  if (d_tsi.length() != 17)
  {
    tt.str("");
    tt << setfill('0') << setw(4) << t_ci.d_mcc << setfill('0') << setw(5) << t_ci.d_mnc << setfill('0') << setw(8) << t_ci.d_issi;
    d_tsi = tt.str();
  }

  t_ci.prio = atoi(h.c_str());

  // store call specific data into a Callinfo struct
  callinfo[t_ci.instance] = t_ci;

  // check if the user is stored? no -> default
  std::map<std::string, User>::iterator iu = userdata.find(o_tsi);
  if (iu == userdata.end())
  {
    Sds t_sds;
    t_sds.direction = OUTGOING;
    t_sds.message = infosds;
    t_sds.tsi = o_tsi;
    t_sds.type = TEXT;
    firstContact(t_sds);
    return;
  }

  uint32_t ti = time(NULL);
  userdata[o_tsi].last_activity = ti;

  // registering user, send to reflector if available
  registerUser(o_tsi);

  // store info in Qso struct
  Qso.tsi = o_tsi;
  Qso.start = ti;

  // prepare event for tetra users to be send over the network
  Json::Value qsoinfo(Json::objectValue);
  qsoinfo["qso_active"] = true;
  qsoinfo["gateway"] = callsign();
  qsoinfo["dest_mcc"] = t_ci.d_mcc;
  qsoinfo["dest_mnc"] = t_ci.d_mnc;
  qsoinfo["dest_issi"] = t_ci.d_issi;
  qsoinfo["aimode"] = t_ci.aistatus;
  qsoinfo["cci"] = t_ci.instance;
  qsoinfo["last_activity"] = ti;
  Qso.members.push_back(iu->second.call);
  qsoinfo["qso_members"] = joinList(Qso.members);
  qsoinfo["active_issi"] = o_tsi;
  qsoinfo["message"] = "Qso:info";
  publishInfo("Qso:info", qsoinfo);

  // callup tcl event
  ss << "groupcall_begin " << t_ci.o_issi << " " << t_ci.d_issi;
  processEvent(ss.str());

  stringstream m_aprsmesg;
  m_aprsmesg << aprspath << ">" << iu->second.call << " initiated groupcall: "
             << t_ci.o_issi << " -> " << t_ci.d_issi;
  log(LOGTRACE, "TetraLogic::handleCallBegin: " + m_aprsmesg.str());
  sendAprs(iu->second.call, m_aprsmesg.str());
} /* TetraLogic::handleCallBegin */


/*
 TETRA SDS Receive +CTSDSR

 CTSDSR unsolicited Result Codes
 +CTSDSR: <AI service>, [<calling party identity>],
 [<calling party identity type>], <called party identity>,
 <called party identity type>, <length>,
 [<end to end encryption>]<CR><LF>user data

 Example:
 +CTSDSR: 12,23404,0,23401,0,112
 (82040801476A61746A616A676A61)
*/
void TetraLogic::handleSds(std::string sds)
{
  log(LOGDEBUG, "***handleSds: " + sds);
  sds.erase(0,9);  // remove "+CTSDSR: "

  // store header of sds for further handling
  uint32_t ti = time(NULL);
  pSDS.aiservice = getNextVal(sds);     // type of SDS (TypeOfService 0-12)
  pSDS.fromtsi = getTSI(getNextStr(sds)); // sender Tsi (23404)
  getNextVal(sds);                      // (0)
  pSDS.totsi = getTSI(getNextStr(sds)); // destination Issi
  getNextVal(sds);                      // (0)
  getNextVal(sds);                      // Sds length (112)
  pSDS.last_activity = ti;
} /* TetraLogic::handleSds */


void TetraLogic::firstContact(Sds tsds)
{
  uint32_t ti = time(NULL);
  userdata[tsds.tsi].issi = tsds.tsi;
  userdata[tsds.tsi].call = "NoCall";
  userdata[tsds.tsi].name = "NoName";
  userdata[tsds.tsi].comment = "no call available for ";
  userdata[tsds.tsi].comment += tsds.tsi;
  userdata[tsds.tsi].idtype = "tsi";
  userdata[tsds.tsi].mode = "TETRA";
  userdata[tsds.tsi].aprs_sym = t_aprs_sym;
  userdata[tsds.tsi].aprs_tab = t_aprs_tab;
  userdata[tsds.tsi].last_activity = ti;

  registerUser(tsds.tsi); // Updsate registration at reflector

  if (infosds.length() > 0)
  {
    tsds.direction = OUTGOING;
    tsds.message = infosds;
    tsds.type = TEXT;
    tsds.remark = "Welcome Sds to a new user";
    log(LOGINFO, "Sending info Sds to new user " + tsds.tsi
         + " \"" + infosds + "\"");
    queueSds(tsds);
  }
} /* TetraLogic::firstContact */


/*
 Handle the sds message
 Example:
 (+CTSDSR: 12,23404,0,23401,0,112)
 82040801476A61746A616A676A61
*/
void TetraLogic::handleSdsMsg(std::string sds)
{
  log(LOGDEBUG, "***handleSdsMsg: " + sds);
  log(LOGTRACE, "TetraLogic::handleSdsMsg: " + sds);
  Sds t_sds;
  stringstream ss, sstcl;
  std::string sds_txt;
  stringstream m_aprsinfo;
  std::map<unsigned int, string>::iterator it;
  LipInfo lipinfo;
  uint32_t ti = time(NULL);
  Json::Value sdsinfo(Json::objectValue);

  t_sds.tos = pSDS.last_activity;      // last activity
  t_sds.direction = INCOMING;          // 1 = received
  t_sds.tsi = pSDS.fromtsi;

  std::map<std::string, User>::iterator iu = userdata.find(t_sds.tsi);
  if (iu == userdata.end())
  {
    firstContact(t_sds);
    return;
  }

  // update last activity of sender
  userdata[t_sds.tsi].last_activity = ti;

  // update registration for reflector
  registerUser(t_sds.tsi);

  t_sds.type = handleMessage(sds);

  unsigned int isds;
  switch (t_sds.type)
  {
    case LIP_SDS:
      handleLipSds(sds, lipinfo);
      m_aprsinfo << "!" << dec2nmea_lat(lipinfo.latitude)
         << iu->second.aprs_sym << dec2nmea_lon(lipinfo.longitude)
         << iu->second.aprs_tab << iu->second.name << ", "
         << iu->second.comment;
      ss << "lip_sds_received " << t_sds.tsi << " "
         << lipinfo.latitude << " " << lipinfo.longitude;
      userdata[t_sds.tsi].lat = lipinfo.latitude;
      userdata[t_sds.tsi].lon = lipinfo.longitude;
      userdata[t_sds.tsi].reasonforsending = lipinfo.reasonforsending;

      // Power-On -> send welcome sds to a new station
      sendWelcomeSds(t_sds.tsi, lipinfo.reasonforsending);

      // send an info sds to all other stations that somebody is in vicinity
      // sendInfoSds(tsi of new station, readonofsending);
      sendInfoSds(t_sds.tsi, lipinfo.reasonforsending);

      // calculate distance RPT<->MS
      sstcl << "distance_rpt_ms " << t_sds.tsi << " "
         << calcDistance(own_lat, own_lon, lipinfo.latitude, lipinfo.longitude)
         << " "
         << calcBearing(own_lat, own_lon, lipinfo.latitude, lipinfo.longitude);
      processEvent(sstcl.str());
      sdsinfo["lat"] = lipinfo.latitude;
      sdsinfo["lon"] = lipinfo.longitude;
      sdsinfo["reasonforsending"] = lipinfo.reasonforsending;
      log(LOGTRACE, "TetraLogic::handleSdsMsg: LIP_SDS: TSI=" + t_sds.tsi
          + ", lat=" + to_string(lipinfo.latitude) + ", lon="
          + to_string(lipinfo.longitude));
      break;

    case STATE_SDS:
      isds = hex2int(sds);
      handleStateSds(isds);
      userdata[t_sds.tsi].state = isds;
      m_aprsinfo << ">" << "State:";
      if ((it = state_sds.find(isds)) != state_sds.end())
      {
        m_aprsinfo << it->second;
      }
      m_aprsinfo << " (" << isds << ")";

      ss << "state_sds_received " << t_sds.tsi << " " << isds;
      sdsinfo["state"] = isds;
      break;

    case TEXT_SDS:
      sds_txt = handleTextSds(sds);
      cfmTxtSdsReceived(sds, t_sds.tsi);
      ss << "text_sds_received " << t_sds.tsi << " \"" << sds_txt << "\"";
      if (!checkIfDapmessage(sds_txt))
      {
        m_aprsinfo << ">" << sds_txt;
      }
      sdsinfo["content"] = sds_txt;
      break;

    case SIMPLE_TEXT_SDS:
      sds_txt = handleSimpleTextSds(sds);
      m_aprsinfo << ">" << sds_txt;
      cfmSdsReceived(t_sds.tsi);
      ss << "text_sds_received " << t_sds.tsi << " \"" << sds_txt << "\"";
      break;

    case ACK_SDS:
      // +CTSDSR: 12,23404,0,23401,0,32
      // 82100002
      // sds msg received by MS from remote
      t_sds.tod = time(NULL);
      sds_txt = handleAckSds(sds, t_sds.tsi);
      m_aprsinfo << ">ACK";
      ss << "sds_received_ack " << sds_txt;
      break;

    case REGISTER_TSI:
      ss << "register_tsi " << t_sds.tsi;
      cfmSdsReceived(t_sds.tsi);
      break;

    case INVALID:
      ss << "unknown_sds_received";
      log(LOGWARN, "*** Unknown type of SDS");
      break;

    default:
      return;
  }

  sdsinfo["last_activity"] = ti;
  sdsinfo["sendertsi"] = t_sds.tsi;
  sdsinfo["type"] = t_sds.type;
  sdsinfo["from"] = userdata[t_sds.tsi].call;
  sdsinfo["to"] = userdata[pSDS.totsi].call;
  sdsinfo["receivertsi"] = pSDS.totsi;
  sdsinfo["gateway"] = callsign();
  sdsinfo["message"] = "Sds:info";
  publishInfo("Sds:info", sdsinfo);

  // send sds info of a user to aprs network
  if (m_aprsinfo.str().length() > 0)
  {
    string m_aprsmessage = aprspath;
    m_aprsmessage += m_aprsinfo.str();
    log(LOGTRACE, m_aprsmessage);
    sendAprs(userdata[t_sds.tsi].call, m_aprsmessage);
  }

  if (ss.str().length() > 0)
  {
    processEvent(ss.str());
  }
} /* TetraLogic::getTypeOfService */


// 6.15.6 TETRA Group Set up
// +CTGS [<group type>], <called party identity> ... [,[<group type>],
//       < called party identity>]
// In V+D group type shall be used. In DMO the group type may be omitted,
// as it will be ignored.
// PEI: +CTGS: 1,09011638300000001
std::string TetraLogic::handleCtgs(std::string m_message)
{
  size_t f = m_message.find("+CTGS: ");
  if ( f != string::npos)
  {
    m_message.erase(0,7);
  }
  return m_message;
} /* TetraLogic::handleCtgs */


/* 6.14.10 TETRA DMO visible gateways/repeaters
 * +CTDGR: [<DM communication type>], [<gateway/repeater address>], [<MNI>],
 *         [<presence information>]
 * TETRA DMO visible gateways/repeaters +CTDGR
 * +CTDGR: 2,1001,90116383,0
 */
std::string TetraLogic::handleCtdgr(std::string m_message)
{
  m_message.erase(0,8);
  stringstream ss, ssret;
  size_t n = std::count(m_message.begin(), m_message.end(), ',');
  DmoRpt drp;
  struct tm mtime = {0};

  if (n == 3)
  {
    int dmct = getNextVal(m_message);
    drp.issi = getNextVal(m_message);
    drp.mni = getNextStr(m_message);
    drp.state = getNextVal(m_message);
    drp.last_activity = mktime(&mtime);

    ssret << "INFO: Station " << TransientComType[dmct] << " detected (ISSI="
          << drp.issi << ", MNI=" << drp.mni << ", state=" << drp.state << ")"
          << endl;
    log(LOGDEBUG, ssret.str());

    dmo_rep_gw.emplace(drp.issi, drp);

    ss << "dmo_gw_rpt " << dmct << " " << drp.issi << " " << drp.mni << " "
       << drp.state;
    processEvent(ss.str());
  }

  return ssret.str();
} /* TetraLogic::handleCtdgr */


void TetraLogic::handleClvl(std::string m_message)
{
  stringstream ss;
  size_t f = m_message.find("+CLVL: ");
  if ( f != string::npos)
  {
    m_message.erase(0,7);
  }

  ss << "audio_level " << getNextVal(m_message);
  log(LOGTRACE, "TetraLogic::handleClvl: " + ss.str());
  processEvent(ss.str());
} /* TetraLogic::handleClvl */


/*
 CMGS Set and Unsolicited Result Code Text
 The set result code only indicates delivery to the MT. In addition to the
 normal <OK> it contains a message reference <SDS instance>, which can be
 used to identify message upon unsolicited delivery status report result
 codes. For SDS-TL messages the SDS-TL message reference is returned. The
 unsolicited result code can be used to indicate later transmission over
 the air interface or the sending has failed.
 +CMGS: <SDS Instance>, [<SDS status>], [<message reference>]
 +CMGS: 0,4,65 <- decimal
 +CMGS: 0
*/
void TetraLogic::handleCmgs(std::string m_message)
{
  std::map<int, Sds>::iterator it;
  size_t f = m_message.find("+CMGS: ");
  if (f != string::npos)
  {
    m_message.erase(0,7);
  }
  int sds_inst = getNextVal(m_message);  // SDS instance
  int state = getNextVal(m_message);     // SDS status: 4 - ok, 5 - failed
  int id = getNextVal(m_message);        // message reference id

  if (last_sdsinstance == sds_inst)
  {
    if (state == SDS_SEND_FAILED)
    {
      log(LOGERROR, "*** ERROR: Sending message failed. Will send again...");
    }
    else if(state == SDS_SEND_OK)
    {
      // MT confirmed the sending of a SDS
      pending_sds.tod = time(NULL); // time of delivery
      for (it=sdsQueue.begin(); it!=sdsQueue.end(); it++)
      {
        if (it->second.id == pending_sds.id)
        {
          it->second = pending_sds;
          log(LOGINFO, "+++ message (" + to_string(it->second.id)
              + ") with ref#" + to_string(id) + " to "
              + it->second.tsi + " successfully sent.");
          break;
        }
      }
    }
    cmgs_received = true;
  }
  //cmgs_received = true;
  last_sdsinstance = sds_inst;
  checkSds();
} /* TetraLogic::handleCmgs */


std::string TetraLogic::handleTextSds(std::string m_message)
{
  log(LOGDEBUG, "***handleTextSds: " + m_message);

  if (m_message.length() > 8) m_message.erase(0,8);  // delete 00A3xxxx
  return decodeSDS(m_message);
} /* TetraLogic::handleTextSds */


string TetraLogic::handleAckSds(string m_message, string tsi)
{
  std::string t_msg;
  t_msg += tsi;
  return t_msg;
} /* TetraLogic::handleAckSds */


std::string TetraLogic::handleSimpleTextSds(std::string m_message)
{
  log(LOGDEBUG, "***handleSimpleTextSds: " + m_message);
  if (m_message.length() > 4) m_message.erase(0,4);  // delete 0201
  return decodeSDS(m_message);
} /* TetraLogic::handleSimpleTextSds */


/*
  6.15.10 Transmission Grant +CTXG
  +CTXG: <CC instance>, <TxGrant>, <TxRqPrmsn>, <end to end encryption>,
         [<TPI type>], [<TPI>]
  e.g.:
  +CTXG: 1,3,0,0,3,09011638300023404
*/
void TetraLogic::handleTxGrant(std::string txgrant)
{
  txgrant.erase(0,7);
  log(LOGTRACE, "TetraLogic::handleTxGrant: " + txgrant);
  if (!is_tx && peistate == OK)
  {
    log(LOGTRACE, "TetraLogic::handleTxGrant: squelchOpen(true)");
    squelchOpen(true);
  }

  current_cci = getNextVal(txgrant);
  getNextVal(txgrant);
  getNextVal(txgrant);
  getNextVal(txgrant);
  getNextVal(txgrant);
  std::string t_tsi = getNextStr(txgrant);

  stringstream ss;
  ss << "tx_grant " << t_tsi;
  processEvent(ss.str());

  // check if the user is stored? no -> default
  std::map<std::string, User>::iterator iu = userdata.find(t_tsi);
  if (iu == userdata.end()) return;

  std::list<std::string>::iterator it;
  it = find(Qso.members.begin(), Qso.members.end(), iu->second.call);
  if (it == Qso.members.end())
  {
    Qso.members.push_back(iu->second.call);
  }

  // registering user, send to reflector if available
  registerUser(t_tsi);
} /* TetraLogic::handleTxGrant */


std::string TetraLogic::getTSI(std::string issi)
{
  stringstream ss;
  char is[18];
  int len = issi.length();
  int t_mcc;
  std::string t_issi;

  if (len < 9)
  {
    sprintf(is, "%08d", atoi(issi.c_str()));
    ss << mcc << mnc << is;
    return ss.str();
  }

  // get MCC (3 or 4 digits)
  if (issi.substr(0,1) == "0")
  {
    t_mcc = atoi(issi.substr(0,4).c_str());
    issi.erase(0,4);
  }
  else
  {
    t_mcc = atoi(issi.substr(0,3).c_str());
    issi.erase(0,3);
  }

  // get ISSI (8 digits)
  t_issi = issi.substr(len-8,8);
  issi.erase(len-8,8);

  sprintf(is, "%04d%05d%s", t_mcc, atoi(issi.c_str()), t_issi.c_str());
  ss << is;

  return ss.str();
} /* TetraLogic::getTSI */


void TetraLogic::handleStateSds(unsigned int isds)
{
  stringstream ss;
  log(LOGINFO, "+++ State Sds received: " + to_string(isds));
  std::map<unsigned int, string>::iterator it = sds_to_command.find(isds);

  if (it != sds_to_command.end())
  {
    // to connect/disconnect Links
    ss << it->second << "#";
    injectDtmf(ss.str(), 10);
  }

  it = state_sds.find(isds);
  if (it != state_sds.end())
  {
    // process macro, if defined
    ss << "D" << isds << "#";
    injectDtmf(ss.str(), 10);
  }
} /* TetraLogic::handleStateSds */


/* 6.15.11 Down Transmission Ceased +CDTXC
 * +CDTXC: <CC instance>, <TxRqPrmsn>
 * +CDTXC: 1,0
 */
void TetraLogic::handleTransmissionEnd(std::string message)
{
  log(LOGTRACE, "TetraLogic::handleTransmissionEnd: " + message);
  log(LOGTRACE, "TetraLogic::handleTransmissionEnd: squelchOpen(false)");
  squelchOpen(false);  // close Squelch
  stringstream ss;
  ss << "groupcall_end";
  processEvent(ss.str());
} /* TetraLogic::handleTransmissionEnd */


// 6.15.3 TETRA Call ReleaseTETRA Call Release
// +CTCR: <CC instance >, <disconnect cause>
// +CTCR: 1,13
void TetraLogic::handleCallReleased(std::string message)
{
  // update Qso information, set time of activity
  log(LOGTRACE, "TetraLogic::handleCallReleased: " + message);
  uint32_t ti = time(NULL);
  Qso.stop = ti;

  stringstream ss;
  message.erase(0,7);
  int cci = getNextVal(message);

  if (rx().squelchIsOpen())
  {
    ss << "out_of_range " << getNextVal(message);
    log(LOGTRACE, "TetraLogic::handleCallReleased: " + ss.str());
  }
  else
  {
    ss << "call_end \"" << DisconnectCause[getNextVal(message)] << "\"";
    log(LOGTRACE, "TetraLogic::handleCallReleased: " + ss.str());
  }
  processEvent(ss.str());
  squelchOpen(false);  // close Squelch

  // send call/qso end to aprs network
  std::string m_aprsmesg = aprspath;
  if (!Qso.members.empty())
  {
    m_aprsmesg += ">Qso ended (";
    m_aprsmesg += joinList(Qso.members);
    m_aprsmesg += ")";

    // prepare event for tetra users to be send over the network
    Json::Value qsoinfo(Json::objectValue);
    qsoinfo["last_activity"] = ti;
    qsoinfo["qso_active"] = false;
    qsoinfo["last_talker"] = callinfo[cci].o_issi;
    qsoinfo["qso_members"] = joinList(Qso.members);
    qsoinfo["gateway"] = callsign();
    qsoinfo["cci"] = cci;
    qsoinfo["aimode"] = callinfo[cci].aistatus;
    qsoinfo["dest_mcc"] = callinfo[cci].d_mcc;
    qsoinfo["dest_mnc"] = callinfo[cci].d_mnc;
    qsoinfo["dest_issi"] = callinfo[cci].d_issi;
    publishInfo("Qso:info", qsoinfo);
    sendAprs(userdata[Qso.tsi].call, m_aprsmesg);
  }
  else
  {
    m_aprsmesg += ">Transmission ended";
    sendAprs(callsign(), m_aprsmesg);
  }
  log(LOGTRACE, "TetraLogic::handleCallReleased: " + m_aprsmesg);

  talkgroup_up = false;
  Qso.members.clear();

  inTransmission = false;

  if (Qso.tsi != "")
  {
    registerUser(Qso.tsi); // updateuserinfo for registration at reflector
  }

  checkSds(); // resend Sds after MS got into Rx mode

} /* TetraLogic::handleCallReleased */


std::string TetraLogic::joinList(std::list<std::string> members)
{
  std::string qi;
  for (const auto &it : members)
  {
    qi += it;
    qi += ",";
  }
  return qi.substr(0,qi.length()-1);
} /* TetraLogic::joinList */


void TetraLogic::sendPei(std::string cmd)
{
  // a sdsmsg must end with 0x1a
  if (cmd.at(cmd.length()-1) != 0x1a)
  {
    cmd += "\r";
  }

  pei->write(cmd.c_str(), cmd.length());

  log(LOGDEBUG, "  To PEI:" + cmd);

  peiComTimer.reset();
  peiComTimer.setEnable(true);
} /* TetraLogic::sendPei */


void TetraLogic::onComTimeout(Async::Timer *timer)
{
  stringstream ss;
  ss << "peiCom_timeout";
  log(LOGTRACE, "TetraLogic::onComTimeout: " + ss.str());
  processEvent(ss.str());
  peistate = TIMEOUT;
} /* TetraLogic::onComTimeout */


void TetraLogic::onPeiActivityTimeout(Async::Timer *timer)
{
  sendPei("AT");
  peirequest = CHECK_AT;
  peiActivityTimer.reset();
} /* TetraLogic::onPeiActivityTimeout */


void TetraLogic::onPeiBreakCommandTimeout(Async::Timer *timer)
{
  peirequest = INIT;
  initPei();
} /* TetraLogic::onPeiBreakCommandTimeout */


/*
  Create a confirmation sds and sends them to the Tetra radio
*/
void TetraLogic::cfmSdsReceived(std::string tsi)
{
  std::string msg("OK");
  Sds t_sds;
  t_sds.message = msg;
  t_sds.tsi = tsi;
  t_sds.direction = OUTGOING;
  log(LOGTRACE, "TetraLogic::cfmSdsReceived: " + tsi);
  queueSds(t_sds);
} /* TetraLogic::cfmSdsReceived */


/* +CTSDSR: 12,23404,0,23401,0,96, 82041D014164676A6D707477 */
void TetraLogic::cfmTxtSdsReceived(std::string message, std::string tsi)
{
  if (message.length() < 8) return;
  std::string id = message.substr(4,2);
  std::string msg("821000");  // confirm a sds received
  msg += id;

  log(LOGINFO, "+++ sending confirmation Sds to " + tsi);

  Sds t_sds;
  t_sds.message = msg;
  t_sds.id = hex2int(id);
  t_sds.remark = "confirmation Sds";
  t_sds.tsi = tsi;
  t_sds.type = ACK_SDS;
  t_sds.direction = OUTGOING;
  queueSds(t_sds);
} /* TetraLogic::cfmSdsReceived */


void TetraLogic::handleCnumf(std::string m_message)
{
  size_t f = m_message.find("+CNUMF: ");
  if (f != string::npos)
  {
    m_message.erase(0,8);
  }
  // e.g. +CNUMF: 6,09011638300023401

  int t_mnc, t_mcc, t_issi;
  short m_numtype = getNextVal(m_message);

  log(LOGINFO, "<num type> is " + to_string(m_numtype)
      + " (" + NumType[m_numtype] + ")");

  if (m_numtype == 6 || m_numtype == 0)
  {
    // get the tsi and split it into mcc,mnc,issi
    splitTsi(m_message, t_mcc, t_mnc, t_issi);

    // check if the configured MCC fits to MCC in MS
    if (t_mcc != atoi(mcc.c_str()))
    {
      log(LOGWARN, "*** ERROR: wrong MCC in MS, will not work! "
             + mcc + "!=" + to_string(t_mcc));
    }

     // check if the configured MNC fits to MNC in MS
    if (t_mnc != atoi(mnc.c_str()))
    {
      log(LOGWARN, "*** ERROR: wrong MNC in MS, will not work! "
             + mnc + "!=" + to_string(t_mnc));
    }
    dmcc = t_mcc;
    dmnc = t_mnc;
    dissi = t_issi;

    if (atoi(issi.c_str()) != t_issi)
    {
      log(LOGWARN, "*** ERROR: wrong ISSI in MS, will not work! "
             + issi + "!=" + to_string(t_issi));
    }
  }

  peirequest = INIT_COMPLETE;
} /* TetraLogic::handleCnumf */


/* format of inject a Sds into SvxLink/TetraLogic
   1) normal: "tsi,message" > /tmp/sds_pty
   e.g. "0901163830023451,T,This is a test"
   2) raw: "tsi,rawmessage" > /tmp/sds_pty
   e.g. "0901163830023451,R,82040102432E4E34E"
*/
void TetraLogic::sdsPtyReceived(const void *buf, size_t count)
{
  const char *buffer = reinterpret_cast<const char*>(buf);
  std::string injmessage = "";

  for (size_t i=0; i<count-1; i++)
  {
    injmessage += *buffer++;
  }
  log(LOGTRACE, "TetraLogic::sdsPtyReceived: " + injmessage);
  string m_tsi = getNextStr(injmessage);
  string type = getNextStr(injmessage);

  // put the new Sds int a queue...
  Sds t_sds;
  t_sds.tsi = m_tsi;
  t_sds.message = injmessage;
  t_sds.direction = OUTGOING;
  t_sds.type = (type == "T" ? TEXT : RAW);
  queueSds(t_sds);

} /* TetraLogic::sdsPtyReceived */


void TetraLogic::sendInfoSds(std::string tsi, short reason)
{
  double timediff;
  float distancediff, bearing;
  stringstream ss, sstcl;
  std::map<std::string, User>::iterator iu = userdata.find(tsi);
  if (iu == userdata.end()) return;

  for (std::map<std::string, User>::iterator t_iu = userdata.begin();
        t_iu != userdata.end(); t_iu++)
  {
    // send info Sds only if
    //    - not the own issi
    //    - not the issi of the dmo-repeater
    //    - time between last sds istn't to short
    //    - is registered at tetra-node
    //
    if (!t_iu->first.empty() && t_iu->first != tsi
          && t_iu->first != getTSI(issi) && t_iu->second.registered)
    {
      timediff = difftime(time(NULL), t_iu->second.sent_last_sds);

      if (timediff >= time_between_sds)
      {
        distancediff = calcDistance(iu->second.lat, iu->second.lon,
                              t_iu->second.lat, t_iu->second.lon);

        bearing = calcBearing(iu->second.lat, iu->second.lon,
                              t_iu->second.lat, t_iu->second.lon);
        ss.str("");
        sstcl.str("");
        ss << iu->second.call << " state change, ";
        if (sds_when_dmo_on && reason == DMO_ON)
        {
          ss << "DMO=on";
          sstcl << "dmo_on " << t_iu->first;
        }
        else if (sds_when_dmo_off && reason == DMO_OFF)
        {
          ss << "DMO=off";
          sstcl << "dmo_off " << t_iu->first;
        }
        else if (sds_when_proximity && distancediff <= proximity_warning)
        {
          ss << "Dist:" << distancediff << "km, Bear:" << std::fixed
             << setprecision(1) << bearing << "deg";
          sstcl << "proximity_info " << t_iu->first << " " << distancediff
                << " " << std::fixed << setprecision(1) << bearing;
        }
        else
        {
          continue;
        }

        // execute tcl procedure(s)
        if (sstcl.str().length() > 0)
        {
          processEvent(sstcl.str());
        }

         // put the new Sds int a queue...
        Sds t_sds;
        t_sds.tsi = t_iu->first;
        t_sds.message = ss.str();
        t_sds.remark = "InfoSds";
        t_sds.direction = OUTGOING;
        t_sds.type = TEXT_SDS;

        log(LOGINFO, "SEND info SDS (to " + t_iu->first + "):" + ss.str());
        // queue SDS
        queueSds(t_sds);
        t_iu->second.sent_last_sds = time(NULL);
      }
    }
  }
} /* TetraLogic::sendInfoSds */


int TetraLogic::handleMessage(std::string mesg)
{
  int retvalue = INVALID;
  typedef std::map<std::string, int> Mregex;
  Mregex mre;

  map<string, int>::iterator rt;

  mre["^OK"]                                      = OK;
  mre["^\\+CME ERROR"]                            = ERROR;
  mre["^\\+CTSDSR:"]                              = SDS;
  mre["^\\+CTICN:"]                               = CALL_BEGIN;
  mre["^\\+CTCR:"]                                = CALL_RELEASED;
  mre["^\\+CTCC:"]                                = CALL_CONNECT;
  mre["^\\+CDTXC:"]                               = TRANSMISSION_END;
  mre["^\\+CTXG:"]                                = TRANSMISSION_GRANT;
  mre["^\\+CTXD:"]                                = TX_DEMAND;
  mre["^\\+CTXI:"]                                = TX_INTERRUPT;
  mre["^\\+CTXW:"]                                = TX_WAIT;
  mre["^\\+CNUM:"]                                = MS_CNUM;
  mre["^\\+CTOM: [0-9]$"]                         = OP_MODE;
  mre["^\\+CMGS:"]                                = CMGS;
  mre["^\\+CNUMF:"]                               = CNUMF;
  mre["^\\+CTGS:"]                                = CTGS;
  mre["^\\+CTDGR:"]                               = CTDGR;
  mre["^\\+CLVL:"]                                = CLVL;
  mre["^\\+CSQ:"]                                 = RSSI;
  mre["^\\+CREG:"]                                = REGISTRATION;
  mre["^\\+GMI:"]                                 = VENDOR;
  mre["^\\+GMM:"]                                 = MODEL;
  mre["^01"]                                      = OTAK;
  mre["^02"]                                      = SIMPLE_TEXT_SDS;
  mre["^03"]                                      = SIMPLE_LIP_SDS;
  mre["^04"]                                      = WAP_PROTOCOL;
  mre["^0A[0-9A-F]{19}"]                          = LIP_SDS;
  mre["^[8-9A-F][0-9A-F]{3}$"]                    = STATE_SDS;
  mre["^8210[0-9A-F]{4}"]                         = ACK_SDS;
  mre["^8[23][0-9A-F]{3,}"]                       = TEXT_SDS;
  //mre["^83"]                                    = LOCATION_SYSTEM_TSDU;
 // mre["^84"]                                    = WAP_MESSAGE;
  mre["^0C"]                                      = CONCAT_SDS;


  for (rt = mre.begin(); rt != mre.end(); rt++)
  {
    if (rmatch(mesg, rt->first))
    {
      retvalue = rt->second;
      return retvalue;
    }
  }

  return peistate;
} /* TetraLogic::handleMessage */


int TetraLogic::getAiMode(std::string aimode)
{
  if (aimode.length() > 6)
  {
    int t = atoi(aimode.erase(0,6).c_str());
    log(LOGINFO, "+++ New Tetra mode: " + AiMode[t]);
    stringstream ss;
    ss << "tetra_mode " << t;
    processEvent(ss.str());
    return t;
  }
  return -1;
} /* TetraLogic::getAiMode */


bool TetraLogic::rmatch(std::string tok, std::string pattern)
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
} /* TetraLogic::rmatch */


// receive interlogic messages here
void TetraLogic::onPublishStateEvent(const string &event_name, const string &msg)
{
  log(LOGTRACE, "TetraLogic::onPublishStateEvent - event_name: "
        + event_name + ", message: " +msg);

  // if it is not allowed to handle information about users then all userinfo
  // traffic will be ignored
  if (!share_userinfo) return;

  Json::CharReaderBuilder builder {};
  auto reader = std::unique_ptr<Json::CharReader>(builder.newCharReader());

  Json::Value user_arr {};
  std::string errors {};
  const auto is_parsed = reader->parse(msg.c_str(),
                                       msg.c_str() + msg.length(),
                                       &user_arr,
                                       &errors);
  if (!is_parsed)
  {
    log(LOGERROR, "*** Error: parsing StateEvent message ("
           + errors + ")" );
    return;
  }

  if (event_name == "DvUsers:info")
  {
    log(LOGDEBUG, "Download userdata from Reflector (DvUsers:info):");
    for (Json::Value::ArrayIndex i = 0; i != user_arr.size(); i++)
    {
      User m_user;
      Json::Value& t_userdata = user_arr[i];
      m_user.issi = t_userdata.get("id", "").asString();
      m_user.idtype = t_userdata.get("idtype", "").asString();
      m_user.mode = t_userdata.get("mode", "").asString();
      m_user.name = t_userdata.get("name", "").asString();
      m_user.call = t_userdata.get("call", "").asString();
      m_user.location = t_userdata.get("location", "").asString();
      m_user.aprs_sym = static_cast<char>(t_userdata.get("sym", 0).asInt());
      m_user.aprs_tab = static_cast<char>(t_userdata.get("tab", 0).asInt());
      m_user.comment = t_userdata.get("comment", "").asString();
      m_user.last_activity = t_userdata.get("last_activity", 0).asUInt();

      userdata[m_user.issi] = m_user;
      log(LOGDEBUG, "Tsi:" + m_user.issi + ", call=" + m_user.call
          + ", name=" + m_user.name + ", location=" + m_user.location
          + ", comment=" + m_user.comment);
    }
  }
  else if (event_name == "ForwardSds:info")
  {
    Json::Value t_msg;
    (user_arr.isArray() ? t_msg = user_arr[0] : t_msg = user_arr);
    std::string destcall = t_msg.get("dest_call", "").asString();
    std::string msg = t_msg.get("sds_info", "").asString();
    std::string source = t_msg.get("source", "").asString();

    for (std::map<std::string, User>::iterator iu = userdata.begin();
       iu != userdata.end(); iu++)
    {
      if (iu->second.call == destcall && iu->second.registered)
      {
        // create a Sds that is forwarded from other node
        Sds t_sds;
        t_sds.tsi = iu->first;
        t_sds.message = source + ":" + msg;
        t_sds.remark = "forwarded Sds from " + source;
        t_sds.direction = OUTGOING;
        t_sds.type = TEXT_SDS;
        queueSds(t_sds);
        log(LOGDEBUG, "Forward Sds from " + source + " to " + destcall
            + ":" + msg);
      }
    }
  }
  else if (event_name == "Request:info")
  {
    sendSystemInfo();
  }
  else if (event_name == "Qso:info")
  {
    // toDo
    Json::Value t_msg;
    (user_arr.isArray() ? t_msg = user_arr[0] : t_msg = user_arr);
    stringstream ss;
    ss << "Got message:" << t_msg.get("name", "").asString() << ","
       << t_msg.get("comment", "").asString() << ","
       << t_msg.get("idtype", "").asString() << ","
       << t_msg.get("tsi", 0).asInt();
    log(LOGTRACE, "TetraLogic::onPublishStateEvent: " + ss.str());
  }
} /* TetraLogic::onPublishStateEvent */


void TetraLogic::publishInfo(std::string type, Json::Value event)
{
  // if it is not allowed to handle information about users then all userinfo traffic
  // will be ignored
  if (!share_userinfo) return;

  // sending own tetra user information to the reflectorlogic network
  Json::StreamWriterBuilder builder;
  log(LOGDEBUG, jsonToString(event));
  builder["commentStyle"] = "None";
  builder["indentation"] = ""; // The JSON document is written on a single line
  Json::StreamWriter* writer = builder.newStreamWriter();
  std::stringstream os;
  writer->write(event, &os);
  delete writer;
  publishStateEvent(type, os.str());
} /* TetraLogic::publishInfo */


int TetraLogic::queueSds(Sds t_sds)
{
  last_sdsid++;
  t_sds.id = last_sdsid;  // last
  t_sds.tos = 0;
  sdsQueue.insert(pair<int, Sds>(last_sdsid, t_sds));
  new_sds = checkSds();
  return last_sdsid;
} /* TetraLogic::queueSds */


void TetraLogic::clearOldSds(void)
{
  vector<int> todelete;
  std::map<int, Sds>::iterator it;

  // delete all old Sds //
  for (it=sdsQueue.begin(); it!=sdsQueue.end(); it++)
  {
    if (it->second.tos != 0 && difftime(it->second.tos, time(NULL)) > inactive_time)
    {
      log(LOGTRACE, "TetraLogic::clearOldSds: " + it->second.tsi
            + "->" + it->second.message);
      todelete.push_back(it->first);
    }
  }
  for (vector<int>::iterator del=todelete.begin(); del!=todelete.end(); del++)
  {
    sdsQueue.erase(sdsQueue.find(*del));
  }
} /* TetraLogic::clearOldSds */


bool TetraLogic::checkSds(void)
{
  std::map<int, Sds>::iterator it;
  bool retsds = false;

  if (sdsQueue.size() < 1) return retsds;

  clearOldSds();

  // if message is sent -> get next available sds message
  if (pending_sds.tod != 0 || (pending_sds.tod == 0 && pending_sds.tos == 0))
  {
    // find the next SDS that was still not send
    for (it=sdsQueue.begin(); it!=sdsQueue.end(); it++)
    {
      if (it->second.tos == 0 && it->second.direction == OUTGOING
          && it->second.nroftries < MAX_TRIES)
      {
        pending_sds = it->second;
        break;
      }
    }
    if (it == sdsQueue.end()) return retsds;
  }

  // now check that the MTM is clean and not in tx state
  if (peistate != OK || inTransmission || rx().squelchIsOpen()) return true;

  if (cmgs_received)
  {
    if (pending_sds.nroftries++ > MAX_TRIES)
    {
      log(LOGERROR, "+++ sending of Sds message failed after "
           + to_string(MAX_TRIES) + " tries, giving up.");
    }
    else
    {
      string t_sds;
      if (pending_sds.type == ACK_SDS)
      {
        createCfmSDS(t_sds, getISSI(pending_sds.tsi), pending_sds.message);
      }
      else
      {
        createSDS(t_sds, getISSI(pending_sds.tsi), pending_sds.message);
      }
      pending_sds.tos = time(NULL);

      log(LOGINFO, "+++ sending Sds (type=" + to_string(pending_sds.type)
             + ") to "
             + getISSI(pending_sds.tsi) + ": \"" + pending_sds.message
             + "\", tries: " + to_string(pending_sds.nroftries));
      sendPei(t_sds);
      cmgs_received = false;
      retsds = true;
    }
  }

  return retsds;
} /* TetraLogic::checkSds */


void TetraLogic::sendWelcomeSds(string tsi, short r4s)
{
  std::map<int, string>::iterator oa = sds_on_activity.find(r4s);

  // send welcome sds to new station, if defined
  if (oa != sds_on_activity.end())
  {
    Sds t_sds;
    t_sds.direction = OUTGOING;
    t_sds.tsi = tsi;
    t_sds.remark = "welcome sds";
    t_sds.message = oa->second;

    log(LOGINFO, "Send SDS:" + getISSI(t_sds.tsi));
    queueSds(t_sds);
  }
} /* TetraLogic::sendWelcomeSds */


/*
 * @param: a message, e.g. +CTCC: 1,1,1,0,0,1,1
 * @return: the current caller identifier
 */
int TetraLogic::handleCci(std::string m_message)
{
  log(LOGTRACE, "TetraLogic::handleCci: " + m_message + ", is_tx=" + (is_tx ? "TRUE":"FALSE"));
  //squelchOpen(true);
  size_t f = m_message.find("+CTCC: ");
  if (f != string::npos)
  {
    m_message.erase(0,7);
    return getNextVal(m_message);
  }
  return 0;
} /* TetraLogic::handleCci */


void TetraLogic::sendAprs(string call, string aprsmessage)
{
  // send group info to aprs network
  if (LocationInfo::has_instance())
  {
    log(LOGINFO, " To APRS:" + aprsmessage);
    LocationInfo::instance()->update3rdState(call, aprsmessage);
  }
} /* TetraLogic::sendAprs */


void TetraLogic::onDapnetMessage(string tsi, string message)
{
  log(LOGINFO, "+++ new Dapnet message received for " + tsi + ":" + message);

  // put the new Sds int a queue...
  Sds t_sds;
  t_sds.tsi = tsi;
  t_sds.remark = "DAPNET message";
  t_sds.message = message;
  t_sds.direction = OUTGOING;
  t_sds.type = TEXT;
  queueSds(t_sds);

  stringstream ss;
  ss << "dapnet_message_received " << tsi << " \"" << message << "\"";
  processEvent(ss.str());

} /* TetraLogic::onDapnetMessage */


void TetraLogic::onDapnetLogmessage(uint8_t type, std::string message)
{
  log(type, message);
} /* TetraLogic::onDapnetLogmessage */


// forward Sds to DAPNet paging service
bool TetraLogic::checkIfDapmessage(std::string message)
{
  string destcall = "";

  if (rmatch(message, "^(to|TO):[0-9A-Za-z]{3,8}:"))
  {
    message.erase(0,3);
  }
  else if (rmatch(message, "^(dap|DAP):[0-9A-Za-z]{3,8}:"))
  {
    message.erase(0,4);
  }
  else
  {
    return false;
  }

  destcall = message.substr(0, message.find(":"));
  message.erase(0, message.find(":")+1);

  // send into DAPNet if registered
  if (dapnetclient)
  {
    log(LOGDEBUG, "To DAPNET: call=" + destcall + ", message:" + message);
    dapnetclient->sendDapMessage(destcall, message);
  }

  Json::Value sds(Json::objectValue);
  sds["dest_callsign"] = destcall;
  sds["sdsmessage"] = message;
  sds["gateway"] = callsign();
  sds["message"] = "ForwardSds:info";
  publishInfo("ForwardSds:info", sds);

  return true;
} /* TetraLogic::checkIfDapmessage */


void TetraLogic::log(uint8_t logtype, std::string logmessage)
{
  if (debug >= logtype)
  {
    cout << logmessage << endl;
  }
} /* TetraLogic::log */


// Pty device that receive AT commands and send it to the "real" Pei
void TetraLogic::peiPtyReceived(const void *buf, size_t count)
{
  const char *buffer = reinterpret_cast<const char*>(buf);
  std::string in = "";

  for (size_t i=0; i<count-1; i++)
  {
    in += *buffer++;
  }
  log(LOGDEBUG, "Command received by Pty device: " + in);

  sendPei(in);
} /* TetraLogic::peiPtyReceived */


void TetraLogic::onQosTimeout(Async::Timer *timer)
{
  getRssi();
} /* TetraLogic::onQosTimeout */


void TetraLogic::getRssi(void)
{
  if (ai == TMO || ai == GATEWAY)
  {
    log(LOGDEBUG, "checking RSSI: AT+CSQ?");
    sendPei("AT+CSQ?");
    qosTimer.reset();
    qosTimer.setEnable(true);
  }
} /* TetraLogic::getRssi */


void TetraLogic::handleRssi(std::string m_message)
{
  size_t f = m_message.find("+CSQ: ");
  if (f != string::npos)
  {
    std::string m = "New Rssi value measured: ";
    stringstream ss;
    ss << "rssi ";
    uint32_t ti = time(NULL);
    m_message.erase(0,6);

    int rssi = getNextVal(m_message);

    checkReg();

    // if =99 no TMO network available
    if (rssi == 99)
    {
      rssi = 0;
      m += "No TMO network connectable or not reliable.";
      log(LOGDEBUG, m);

      ss << "-127";
      processEvent(ss.str());
      return;
    }

    rssi = -113 + 2 * rssi;
    rssi_list.push_back(rssi);
    if (rssi_list.size() > 20) rssi_list.erase(rssi_list.begin());
    if (rssi < min_rssi) min_rssi = rssi; // store min rssi value
    if (rssi > max_rssi) max_rssi = rssi; // store max rssi value

    ss << rssi;
    processEvent(ss.str());

    m += to_string(rssi);
    m += " dBm (";
    m += getRssiDescription(rssi);
    m +=").";
    log(LOGDEBUG, m);

    // send the rssi value to the refelctor network for further handling
    Json::Value t_rssi(Json::objectValue);
    t_rssi["issi"] = dissi;
    t_rssi["mni"] = reg_mni;
    t_rssi["call"] = callsign();
    t_rssi["la"] = reg_la;
    t_rssi["last_activity"] = ti;
    t_rssi["rssi"] = rssi;
    t_rssi["max_rssi"] = *max_element(rssi_list.begin(), rssi_list.end());
    t_rssi["min_rssi"] = *min_element(rssi_list.begin(), rssi_list.end());
    t_rssi["message"] = "Rssi:info";
    t_rssi["last_activity"] = ti;
    publishInfo("Rssi:info", t_rssi);

    // no action if the value is above the defined limit
    if (rssi > qos_limit) return;

    // send email (via the tcl framework) to the administrator
    if (qos_email_to.length() > 5)
    {
      string s = "rssi_limit ";
      s += to_string(rssi);
      s += " ";
      s += getRssiDescription(rssi);
      s += " ";
      s += qos_email_to;
      processEvent(s);
    }

    // send sds to the administrator
    if (qos_sds_to.length() > 1)
    {
      string sds;
      string s = "New Rssi limit: ";
      s += to_string(rssi);
      s += " dBm (";
      s += getRssiDescription(rssi);
      s += ").";
      Sds t_sds;
      t_sds.direction = OUTGOING;
      t_sds.tsi = qos_sds_to;
      t_sds.message = s;
      t_sds.type = TEXT;
      t_sds.remark = "Rssi-Sds";
      queueSds(t_sds);
      log(LOGDEBUG, "Sending SDS: " + s);
    }
  }
} /* TetraLogic::handleRssi */


void TetraLogic::checkReg(void)
{
  log(LOGDEBUG, "Checking registration state (AT+CREG?)");
  sendPei("AT+CREG?");
} /* TetraLogic::checkReg */


void TetraLogic::handleCreg(std::string m_message)
{
  // +CREG: 1,1,90109999
  if (m_message.length()>5) m_message.erase(0,6);
  reg_state = getNextVal(m_message);
  reg_la = getNextVal(m_message); // 14-bit Location Area code
  reg_mni = getNextVal(m_message); // 24-bit Mobile Network Identity
  stringstream ss;
  ss << "Registration LA=" << reg_la << ", MNI=" <<
     reg_mni << ", state=" << RegStat[reg_state];
  log(LOGDEBUG, ss.str());

  string s = "registration_state " + reg_la;
  s += " ";
  s += reg_mni;
  s += " ";
  s += reg_state;
  processEvent(s);
} /* TetraLogic::handleCreg */


void TetraLogic::handleModel(std::string m_message)
{
  // +GMM: 54007,M83PF-----AN,88.2.0.0
  m_message.erase(0,6);
  model = getNextStr(m_message);
} /* TetraLogic::handleModel */


void TetraLogic::handleVendor(std::string m_message)
{
  // +GMI: MOTOROLA
  m_message.erase(0,6);
  vendor = m_message;
} /* TetraLogic::handleVendor */


void TetraLogic::sendSystemInfo(void)
{
  // prepare event systeminfo for reflector
  Json::Value systeminfo(Json::objectValue);
  systeminfo["vendor"] = vendor;
  systeminfo["model"] = model;
  systeminfo["call"] = callsign();
  systeminfo["issi"] = issi;
  systeminfo["message"] = "System:info";
  systeminfo["tl_version"] = TETRA_LOGIC_VERSION;
  publishInfo("System:info", systeminfo);
} /* TetraLogic::sendSystemInfo */


// register an active user, "active" means that SvxLink has registered an
// activity within the last "inactive_time" seconds (sds, ptt, messages or whatever)
void TetraLogic::registerUser(std::string tsi)
{
  Json::Value event(Json::arrayValue);
  time_t ti = time(NULL);

  // update user list (set to active or not active)
  std::map<std::string, User>::iterator iu = userdata.find(tsi);
  if (iu != userdata.end())
  {
    iu->second.registered = true;
    iu->second.last_activity = ti;
  }

  for (iu = userdata.begin(); iu != userdata.end(); iu++)
  {
    Json::Value t_userinfo(Json::objectValue);
    if (iu->second.registered)
    {
      t_userinfo["tsi"] = iu->second.issi;
      t_userinfo["idtype"] = iu->second.idtype;
      t_userinfo["call"] = iu->second.call;
      t_userinfo["mode"] = iu->second.mode;
      t_userinfo["name"] = iu->second.name;
      t_userinfo["tab"] = iu->second.aprs_tab;
      t_userinfo["sym"] = iu->second.aprs_sym;
      t_userinfo["comment"] = iu->second.comment;
      t_userinfo["location"] = iu->second.location;
      t_userinfo["last_activity"] = (uint32_t)iu->second.last_activity;
      t_userinfo["registered"] = iu->second.registered;
      t_userinfo["message"] = "Register:info";
      event.append(t_userinfo);
    }
  }

  publishInfo("Register:info", event);
  checkUserReg();
} /* TetraLogic::registerUser */


void TetraLogic::userRegTimeout(Async::Timer *timer)
{
  checkUserReg();
  userRegTimer.reset();
  userRegTimer.setEnable(true);
} /* TetraLogic::userRegTimeout */


void TetraLogic::checkUserReg(void)
{
  time_t ti = time(NULL);

  // update user list (set to active or not active)
  std::map<std::string, User>::iterator iu;

  for (iu = userdata.begin(); iu != userdata.end(); iu++)
  {
    if (iu->second.registered && iu->second.last_activity < ti - inactive_time)
    {
      iu->second.registered = false; // set to not active if >inactive_time secs
      stringstream so;
      so << "+++ CheckUserRegistration:" << iu->second.issi
         << " is now unregistered." << endl;
      log(LOGDEBUG, so.str());
    }
  }
} /* TetraLogic::checkUserReg */


void TetraLogic::sendSds(const std::string& issi, const std::string& message)
{
  stringstream ss;
  ss << "Sending Sds from Tcl to " << issi << ", message=" << message;
  log(LOGINFO, ss.str());

  Sds t_sds;
  t_sds.direction = OUTGOING;
  t_sds.tsi = issi;
  t_sds.message = message;
  t_sds.type = TEXT;
  t_sds.remark = "by Tcl procedur sendSds";
  queueSds(t_sds);
} /* TetraLogic::sendSds */


void TetraLogic::setupCall(const std::string& issi)
{
  stringstream ss;
  ss << "Setup call from Tcl to " << issi;
  log(LOGINFO, ss.str());

  initGroupCall(atoi(issi.c_str()));

} /* TetraLogic::sendSds */


bool TetraLogic::getConfigValue(const std::string& section,
                               const std::string& tag,
                               std::string& value)
{
  return cfg().getValue(section, tag, value, true);
} /* TetraLogic::getConfigValue */


void TetraLogic::dummy(const std::string& info)
{
  cout << "Tetra-Logic::dummy: " << info << endl;
} /* TetraLogic::dummy */


std::string TetraLogic::jsonToString(Json::Value eventmessage)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  std::string message = Json::writeString(builder, eventmessage);
  return message;
} /* Reflector::jsonToString */

/*
 * This file has not been truncated
 */
