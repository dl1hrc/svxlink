###############################################################################
#
# SelCallEnc module implementation
#
###############################################################################

#
# This is the namespace in which all functions and variables below will exist.
# The name must match the configuration variable "NAME" in the
# [ModuleSelCallEnc] section in the configuration file. The name may be changed
# but it must be changed in both places.
#
namespace eval SelCallEnc {

#
# Extract the module name from the current namespace
#
set module_name [namespace tail [namespace current]];


#
# Type ID to selcall type name mapping
#
array set variants {
  "01" "ZVEI1"
  "02" "ZVEI2"
  "03" "ZVEI3"
  "04" "PZVEI"
  "05" "DZVEI"
  "06" "EEA"
  "07" "CCIR1"
  "08" "CCIR2"
  "09" "VDEW"
  "10" "CCITT"
  "11" "NATEL"
  "12" "EIA"
  "13" "EURO"
  "14" "MODAT"
  "15" "PDZVEI"
  "16" "PCCIR"
  "17" "AUTOA"
  "18" "QC2"
}


#
# A convenience function for printing out information prefixed by the
# module name
#
#   msg - The message to print
#
proc printInfo {msg} {
  variable module_name
  puts "$module_name: $msg"
}


#
# A convenience function for calling an event handler
#
#   ev - The event string to execute
#
proc processEvent {ev} {
  variable module_name
  ::processEvent "${module_name}" "$ev"
}


#
# Get the variant name given the two digit id. If the variant is not found, an
# empty string is returned.
#
#   number - The two digit id number
#
proc getVariant {number} {
  variable variants

  if [info exists variants($number)] {
    return $variants($number)
  }
  return ""
}


#
# Emit the play_help event on the special format used by this module
#
proc playHelp {} {
  variable variants

  # play_help ID1=NAME1 ID2=NAME2 ...
  set ev play_help
  foreach variant_id [lsort [array names variants]] {
    append ev " $variant_id=$variants($variant_id)"
  }
  processEvent "$ev"
}


#
# Executed when this module is being activated
#
proc activateInit {} {
  printInfo "Module activated"
}


#
# Executed when this module is being deactivated.
#
proc deactivateCleanup {} {
  #printInfo "Module deactivated"
}


#
# Executed when a DTMF digit (0-9, A-F, *, #) is received
#
#   char - The received DTMF digit
#   duration - The duration of the received DTMF digit
#
proc dtmfDigitReceived {char duration} {
  #printInfo "DTMF digit $char received with duration $duration milliseconds"
}


#
# Executed when a DTMF command is received
#
#   cmd - The received DTMF command
#
proc dtmfCmdReceived {cmd} {
  printInfo "DTMF command received: $cmd";

  if {$cmd == "0"} {
    playHelp
  } elseif {$cmd == ""} {
    deactivateModule
  } elseif {[string length $cmd] >= 2 && \
            [set variant [getVariant [string range $cmd 0 1]]] != ""} {
    if {[string length $cmd] > 2} {
      processEvent "play_sel_call $variant [string range $cmd 2 end]"
    } else {
      processEvent "play_standard $variant"
    }
  } else {
    processEvent "unknown_command $cmd"
  }
}


#
# Executed when a DTMF command is received in idle mode. That is, a command is
# received when this module has not been activated first.
#
#   cmd - The received DTMF command
#
proc dtmfCmdReceivedWhenIdle {cmd} {
  printInfo "DTMF command received when idle: $cmd";

  if {$cmd == "0"} {
    playHelp
  } elseif {[string length $cmd] >= 2 && \
            [set variant [getVariant [string range $cmd 0 1]]] != ""} {
    if {[string length $cmd] > 2} {
      processEvent "play_sel_call $variant [string range $cmd 2 end]"
    } else {
      processEvent "play_standard $variant"
    }
  } else {
    processEvent "unknown_command $cmd"
  }
}


#
# Executed when the squelch open or close.
#
#   is_open - Set to 1 if the squelch is open otherwise it's set to 0
#
proc squelchOpen {is_open} {
  #if {$is_open} {set str "OPEN"} else {set str "CLOSED"};
  #printInfo "The squelch is $str"
}


#
# Executed when all announcement messages has been played.
# Note that this function also may be called even if it wasn't this module
# that initiated the message playing.
#
proc allMsgsWritten {} {
}


# end of namespace
}

#
# This file has not been truncated
#

