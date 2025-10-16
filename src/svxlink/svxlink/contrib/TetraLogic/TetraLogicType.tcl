###############################################################################
#
# TetraLogic event handlers
#
###############################################################################

#
# This is the namespace in which all functions below will exist. The name
# must match the corresponding section "[TetraLogic]" in the configuration
# file. The name may be changed but it must be changed in both places.
#
namespace eval ${::logic_name} {

sourceTclWithOverrides "Logic.tcl";
mixin Logic;

#
# defines the eMail address of the sender ("From:")
# Note: ssmtp is needed for functionality
#
variable from "test@test.com";


#
# Executed when a specified module could not be found
#
proc no_such_module {module_id} {

}


#
# Executed when a manual identification is initiated with the * DTMF code
#
proc manual_identification {} {

}


#
# Executed when the squelch just have closed and the RGR_SOUND_DELAY timer has
# expired.
#
proc send_rgr_sound {} {

}


#
# Executed when an empty macro command (i.e. D#) has been entered.
#
proc macro_empty {} {

}


#
# Executed when an entered macro command could not be found
#
proc macro_not_found {} {

}


#
# Executed when a macro syntax error occurs (configuration error).
#
proc macro_syntax_error {} {

}


#
# Executed when the specified module in a macro command is not found
# (configuration error).
#
proc macro_module_not_found {} {

}


#
# Executed when the activation of the module specified in the macro command
# failed.
#
proc macro_module_activation_failed {} {

}


#
# Executed when a macro command is executed that requires a module to
# be activated but another module is already active.
#
proc macro_another_active_module {} {

}


#
# Executed when an unknown DTMF command is entered
#
proc unknown_command {cmd} {

}


#
# Executed when an entered DTMF command failed
#
proc command_failed {cmd} {

}


#
# Executed once every whole minute
#
proc every_minute {} {
}


#
# Executed once every second
#
proc every_second {} {

}


#
# Executed each time the transmitter is turned on or off
#
proc transmit {is_on} {

}


#
# Executed each time the squelch is opened or closed
#
proc squelch_open {rx_id is_open} {

}


#
# Executed when a DTMF digit has been received
#   digit     - The detected DTMF digit
#   duration  - The duration, in milliseconds, of the digit
#
# Return 1 to hide the digit from further processing in SvxLink or
# return 0 to make SvxLink continue processing as normal.
#
proc dtmf_digit_received {digit duration} {

}


#
# Executed when a DTMF command has been received
#   cmd - The command
#
# Return 1 to hide the command from further processing is SvxLink or
# return 0 to make SvxLink continue processing as normal.
#
proc dtmf_cmd_received {cmd} {

}


#
# Executed when the QSO recorder is being activated
#
proc activating_qso_recorder {} {

}


#
# Executed when the QSO recorder is being deactivated
#
proc deactivating_qso_recorder {} {

}


#
# Executed when trying to deactivate the QSO recorder even though it's
# not active
#
proc qso_recorder_not_active {} {

}


#
# Executed when trying to activate the QSO recorder even though it's
# already active
#
proc qso_recorder_already_active {} {

}


#
# Executed when the timeout kicks in to activate the QSO recorder
#
proc qso_recorder_timeout_activate {} {

}


#
# Executed when the timeout kicks in to deactivate the QSO recorder
#
proc qso_recorder_timeout_deactivate {} {

}


#
# Executed when the user is requesting a language change
#
proc set_language {lang_code} {

}


#
# Executed when Pei communication fails
#
proc peiCom_timeout {} {
  puts "*** ERROR: No or wrong response on Pei command";
}


#
# Executed when the user requests a list of available languages
#
proc list_languages {} {

}


#
# Executed when the Pei initializing process has been finished
#
proc pei_init_finished {} {
  puts "PEI init finished";
}


#
# Executed when the node is being brought online after being offline
#
proc logic_online {online} {

}


#
# Executed when a Sds header has been received
#
proc sds_header_received {issi type} {
}


#
# Executed when a Sds has been received
#
proc sds_message {sds} {
  puts $sds;
}


#
# Executed when a groupcall has been started
#
proc groupcall_begin {issi dissi} {
  puts "Groupcall from $issi to $dissi";
}


#
# Executed when transmission is granted
#
proc tx_grant {tsi} {
}


#
# Executed when a groupcall is ended
#
proc groupcall_end {} {
}


#
# Executed when a call is ended
#
proc call_end {reason} {
}


#
# Executed when a state sds has been received
#
proc state_sds_received {tsi state} {
}


#
# Executed when a Lip sds has been received
# TSI - tsi of the sender
# lat - the latitude of the sender in decimal/float
# lon - the longitude of the sender in decimal/float
proc lip_sds_received {tsi lat lon} {
}


#
# Executed when a text sds has been received
#
proc text_sds_received {tsi message} {
 # puts "SDS received from $tsi: $message"
}


#
# Executed when a tsi is registering
#
proc register_tsi {tsi} {
}


#
# Executed when an unknown Sds is received
#
proc unknown_sds_received {} {
}


#
# Executed when an groupcall is initiated
#
proc init_group_call {gssi} {
  puts "Init groupcall to GSSI: $gssi"
}


#
# Executed when a sds reception has been acknowledged
#
proc sds_received_ack {message} {

}


#
# Executed when a station is registered
#
proc register_station {tsi} {

}


#
# Executed when a station is switching DMO-mode=on
#
proc dmo_on {tsi} {

}


#
# Executed when a station is switching DMO-mode=off
#
proc dmo_off {tsi} {

}


#
# Executed when a station is in the vicinity
#
proc proximity_info {tsi distance bearing} {

}


#
# Executed each time a distance between mobile station and repeater is calculated
#
proc distance_rpt_ms {tsi distance bearing} {

}


#
# Executed when the MS sends a message about the state of a Sds
# +CMGS= <called party identity >, <length><CR><LF>user data<CtrlZ> 
#
proc sent_message {response} {

}


#
# Executed when a visible DmoRepeater or gateway is located in the vicinity
# [<DM communication type>], [<gateway/repeater address>], [<MNI>],
#             [<presence information>]
proc dmo_gw_rpt {dmct gw_rpt_issi mni pi} {

}


#
# Executed when the audio level of the Rx has been changed
#
proc audio_level {lvl} {

}


#
# Executed if the squelch has not be closed by call-end but by e.g. out of
# range or transmission has been stopped by low battery
#
proc out_of_range {dc} {

}


#
# Executed if the tetra mode has changed (AI mode)
#
proc tetra_mode {aimode} {

}


#
# Executed when a talkgroup is received from another linked logic core
#
#   logic -- The name of the logic core
#   tg    -- The received talkgroup
#
proc remote_received_tg_updated {logic tg} {

}


#
# Executed when a new RSSI value has been measured
#
proc rssi {rssi} {
  puts "current RSSI: $rssi dBm";
  sendSds 2629143 "Current RSSI=$rssi dBm";
}


#
#
#
proc siglev_updated {rx level} {

}


#
# Executed when the RSSI value exceeds a defined limit
#
proc rssi_limit {rssi description mail} {
  variable from;
  set cmd "|/usr/sbin/ssmtp $mail";
  set to "To:$mail";
  set subject "Subject:WARNING - low RSSI $rssi";
  set message "The RSSI value has fallen below $rssi dBm ($description).";

  set m [open $cmd w];
    puts $m $from;
    puts $m $to;
    puts $m $subject;
    puts $m $message;
  close $m;
}


#
# Executed if the registration state has changed
#
proc registration_state {la mni state} {

}


#
# Executed if a Dapnet message was received
#
proc dapnet_message_received {tsi message} {
}


#
# Examples for use of new procedures
#
proc example {} {
  sendSds 12345 "Testmessage";
  setupCall 12345;
}

#
#
#
proc startup {} {
}


#
#
#
proc dummy {} {
}

# end of namespace
}


#
# This file has not been truncated
#
