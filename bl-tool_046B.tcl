#!/usr/bin/wish
#
# Copyright by Oliver Dippel <oliver@multixmedia.org>
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version. see <http://www.gnu.org/licenses/>
# 

set VERSION 46


wm title . "Brushless-Gimbal-Tool (for v$VERSION)"

if {[string match "*Linux*" $tcl_platform(os)]} {
	set comports ""
	set device ""
	catch {
		set comports [glob /dev/ttyUSB*]
		set device "[lindex $comports end]"
	}
} elseif {[string match "*Windows*" $tcl_platform(os)]} {
	set comports {"com1:" "com2:" "com3:" "com4:" "com5:" "com6:" "com7:" "com8:" "com9:" "com10:" "com11:" "com12:" "com13:" "com14:" "com15:"}
	catch {
		set serial_base "HKEY_LOCAL_MACHINE\\HARDWARE\\DEVICEMAP\\SERIALCOMM"
		set values [registry values $serial_base]
		set res {}
		foreach valueName $values {
			set PortName [registry get $serial_base $valueName]
			lappend res "$PortName:"
		}
		set comports $res
	}
	set device "[lindex $comports end]"
} elseif {[string match "*Darwin*" $tcl_platform(os)] || [string match "*MacOS*" $tcl_platform(os)]} {
	set comports ""
	set device ""
	catch {
		set comports [glob /dev/cu.usbserial-*]
		set device "[lindex $comports end]"
	}
}


set Serial 0

set LastValX 0
set LastValY 0

set gyroPitchKp 0.0
set gyroPitchKi 0.0
set gyroPitchKd 0.0
set motorPitchMaxpwm 0
set motorPitchNumber 0
set motorPitchPoles 0
set motorPitchDir 0

set gyroRollKp 0.0
set gyroRollKi 0.0
set gyroRollKd 0.0
set motorRollMaxpwm 0
set motorRollNumber 1
set motorRollPoles 0
set motorRollDir 0

set accelWeight 0.0
set motorUpdateFreq 0
set use_ACC 0

set rcGain 0

proc Serial_Init {ComPort ComRate} {
	global Serial
	catch {close $Serial}
	catch {fileevent $Serial readable ""}
	set iChannel 0
	if {[catch {
		set iChannel [open $ComPort w+]
		fconfigure $iChannel -mode $ComRate,n,8,1 -ttycontrol {RTS 1 DTR 0} -blocking FALSE
		fileevent $iChannel readable [list rd_chid $iChannel]
		.version configure -text "Serial-Ok: $ComPort @ $ComRate"
		.info configure -text "Serial-Ok: $ComPort @ $ComRate"
		.device.connect configure -text "Reconnect"
		enable_all .device.close
	}]} {
		.version configure -text "Serial-Error: $ComPort @ $ComRate"
		.version configure -background red
		.info configure -text "Serial-Error: $ComPort @ $ComRate"
		.device.connect configure -text "Connect"
		disable_all .device.close
		return 0
	}
	return $iChannel
}

set buffer ""
set mode "OAC"
set count 0
set chart_count 0

proc close_serial {} {
	global Serial
	catch {close $Serial}
	.device.connect configure -text "Connect"

	disable_all .motor
	disable_all .general.chart
	disable_all .general.settings
	disable_all .buttons
	disable_all .device.close

}


proc connect_serial {} {
	global Serial
	global mode
	global count
	global device
	global buffer

	.info configure -background yellow

	set device [.device.spin get]
	set Serial [Serial_Init $device 115200]
	set mode "OAC"
	set count 0

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		set mode ""
		return
	}

	after 100 send_tc
}

proc send_tc {} {
	global Serial
	global mode
	global count
	global device
	global buffer

	set count 0
	set buffer ""

	.info configure -background yellow

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		set mode ""
		return
	}

	if {$mode == "OAC" || $mode == "ODM"} {
#		puts "## reset chart-data ##"
	      	puts -nonewline $Serial "ODM 0\n"
		flush $Serial
	      	puts -nonewline $Serial "OAC 0\n"
		flush $Serial
		after 1500
		set NULL [read $Serial 10000]
		flush $Serial
		set NULL [read $Serial 10000]
		flush $Serial
		set NULL [read $Serial 10000]
		flush $Serial
		set buffer ""

		.general.chart.button configure -relief raised
		.general.chart.button configure -text "Start"
	}

	.info configure -text "TC: reading values..."

	set mode "TC"
        puts -nonewline $Serial "TC\n"
	flush $Serial
}

proc send_trc {} {
	global Serial
	global mode
	global count
	global device

	set mode "TRC"
	set count 0
	set buffer ""

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		set mode ""
		return
	}

	.info configure -text "TRC: reading values..."

        puts -nonewline $Serial "TRC\n"
	flush $Serial
}

proc send_tac {} {
	global Serial
	global mode
	global count
	global device

	set mode "TAC"
	set count 0
	set buffer ""

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		set mode ""
		return
	}

	.info configure -text "TAC: reading values..."

        puts -nonewline $Serial "TAC\n"
	flush $Serial
}

proc send_tca {} {
	global Serial
	global mode
	global count
	global device

	set mode "TCA"
	set count 0
	set buffer ""

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		set mode ""
		return
	}

	.info configure -text "TCA: reading values..."

        puts -nonewline $Serial "TCA\n"
	flush $Serial
}

proc send_trg {} {
	global Serial
	global mode
	global count
	global device

	set mode "TRG"
	set count 0
	set buffer ""

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		set mode ""
		return
	}

	.info configure -text "TRG: reading values..."

        puts -nonewline $Serial "TRG\n"
	flush $Serial
}

proc draw_chart {} {
	global Serial
	global mode
	global count
	global device
	global use_ACC

	set count 0
	set buffer ""
	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		set mode ""
		return
	}

	flush $Serial

	if {$mode == "ODM" || $mode == "OAC"} {
        	puts -nonewline $Serial "ODM 0\n"
		flush $Serial
        	puts -nonewline $Serial "OAC 0\n"
		flush $Serial
		set mode ""
		.general.chart.button configure -relief raised
		.general.chart.button configure -text "Start"
		.info configure -text "stopping stream"
		.info configure -background green

		enable_all .motor
		enable_all .general.chart
		enable_all .general.settings
		enable_all .buttons

		update
	} else {
		if {$use_ACC == 0} {
			set mode "ODM"
		        puts -nonewline $Serial "ODM 1\n"
		} else {
			set mode "OAC"
		        puts -nonewline $Serial "OAC 1\n"
		}
		flush $Serial
		.general.chart.button configure -relief sunken
		.general.chart.button configure -text "Stop"
		.info configure -text "starting stream"
		.info configure -background yellow

		disable_all .motor
		disable_all .general.settings
		disable_all .buttons

		update
	}
}

proc save_values {} {
	global Serial
	global mode
	global count
	global device

	global gyroPitchKp
	global gyroPitchKi
	global gyroPitchKd
	global motorPitchMaxpwm
	global motorPitchNumber
	global motorPitchPoles
	global motorPitchDir
	global rcPitchMin
	global rcPitchMax
	global gyroRollKp
	global gyroRollKi
	global gyroRollKd
	global motorRollMaxpwm
	global motorRollNumber
	global motorRollPoles
	global motorRollDir
	global rcRollMin
	global rcRollMax
	global accelWeight
	global motorUpdateFreq
	global use_ACC
	global rcAbsolute
	global rcGain

	.info configure -text "saving values"
	.info configure -background yellow
	update

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		return
	}

        puts -nonewline $Serial "SP [expr $gyroPitchKp * 1000.0] [expr $gyroPitchKi * 1000.0] [expr $gyroPitchKd * 1000.0]\n"
	flush $Serial
	after 100
        puts -nonewline $Serial "SR [expr $gyroRollKp * 1000.0] [expr $gyroRollKi * 1000.0] [expr $gyroRollKd * 1000.0]\n"
	flush $Serial
	after 100
        puts -nonewline $Serial "SA [expr $accelWeight * 10000.0]\n"
	flush $Serial
	after 100
        puts -nonewline $Serial "SF $motorPitchPoles $motorRollPoles\n"
	flush $Serial
	after 100
        puts -nonewline $Serial "SE $motorPitchMaxpwm $motorRollMaxpwm\n"
	flush $Serial
	after 100
	if {$motorPitchDir == 0} {
		set motorPitchDir2 -1
	} else {
		set motorPitchDir2 1
	}
	if {$motorRollDir == 0} {
		set motorRollDir2 -1
	} else {
		set motorRollDir2 1
	}
        puts -nonewline $Serial "SM $motorPitchDir2 $motorRollDir2 $motorPitchNumber $motorRollNumber\n"
	flush $Serial
	after 100
        puts -nonewline $Serial "SRC $rcPitchMin $rcPitchMax $rcRollMin $rcRollMax\n"
	flush $Serial
	after 100
        puts -nonewline $Serial "UAC $use_ACC\n"
	flush $Serial
	after 100
        puts -nonewline $Serial "SCA $rcAbsolute\n"
	flush $Serial
	after 100
        puts -nonewline $Serial "SRG $rcGain\n"
	flush $Serial

	after 100 send_tc
}

proc gyro_cal {} {
	global Serial
	global mode
	global count
	global device

	set mode ""
	set count 0

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		return
	}


        puts -nonewline $Serial "GC\n"
	flush $Serial
}

proc save_to_flash {} {
	global Serial
	global mode
	global count
	global device

	set mode ""
	set count 0

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		return
	}

        puts -nonewline $Serial "WE\n"
	flush $Serial
}

proc load_from_flash {} {
	global Serial
	global mode
	global count
	global device
	set mode ""
	set count 0

	.info configure -text "loading from flash"
	.info configure -background yellow
	update

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		return
	}

        puts -nonewline $Serial "RE\n"
	flush $Serial
	after 100

	after 100 send_tc
}

proc set_defaults {} {
	global Serial
	global mode
	global count
	global device
	set mode ""
	set count 0

	.info configure -text "setting defaults"
	.info configure -background yellow
	update

	if {$Serial == 0} {
		.info configure -background red
		.info configure -text "not connected"
		return
	}

        puts -nonewline $Serial "SD\n"
	flush $Serial
	after 100

	after 100 send_tc
}


proc rd_chid {chid} {
	global buffer
	global mode
	global count
	global chart_count
	global gyroPitchKp
	global gyroPitchKi
	global gyroPitchKd
	global motorPitchMaxpwm
	global motorPitchNumber
	global motorPitchPoles
	global motorPitchDir
	global rcPitchMin
	global rcPitchMax
	global gyroRollKp
	global gyroRollKi
	global gyroRollKd
	global motorRollMaxpwm
	global motorRollNumber
	global motorRollPoles
	global motorRollDir
	global rcRollMin
	global rcRollMax
	global accelWeight
	global motorUpdateFreq
	global use_ACC
	global rcAbsolute
	global VERSION
	global rcGain

	if {$chid == 0} {
		return
	}
	catch {
		set ch [read $chid 1]

		if {$ch == "\n"} {

#			puts "$mode - #$buffer#"

			if {$mode == "TC"} {

				if {[string match "*GO!*" $buffer] || [string match "*MPU6050*" $buffer]} {
					set buffer ""
					return
				}

				.info configure -text "TC: reading values...([expr $count + 1]/16): $buffer"
				if {[string is integer -strict $buffer]} {
					if {$count == 0} {
						if {$buffer == $VERSION} {
							.version configure -text "Firmware-Version: $buffer"
							.version configure -background lightgray
						} else {
							.version configure -text "Firmware-Version: $buffer (wrong Version)"
							.version configure -background red
						}
					} elseif {$count == 1} {
						set gyroPitchKp [expr $buffer / 1000.0]
					} elseif {$count == 2} {
						set gyroPitchKi [expr $buffer / 1000.0]
					} elseif {$count == 3} {
						set gyroPitchKd [expr $buffer / 1000.0]
					} elseif {$count == 4} {
						set gyroRollKp [expr $buffer / 1000.0]
					} elseif {$count == 5} {
						set gyroRollKi [expr $buffer / 1000.0]
					} elseif {$count == 6} {
						set gyroRollKd [expr $buffer / 1000.0]
					} elseif {$count == 7} {
						set accelWeight [expr $buffer / 10000.0]
					} elseif {$count == 8} {
						set motorPitchPoles $buffer
					} elseif {$count == 9} {
						set motorRollPoles $buffer
					} elseif {$count == 10} {
						if {$buffer == -1} {
							set motorPitchDir 0
						} else {
							set motorPitchDir 1
						}
					} elseif {$count == 11} {
						if {$buffer == -1} {
							set motorRollDir 0
						} else {
							set motorRollDir 1
						}
					} elseif {$count == 12} {
						set motorPitchNumber $buffer
					} elseif {$count == 13} {
						set motorRollNumber $buffer
					} elseif {$count == 14} {
						set motorPitchMaxpwm $buffer
					} elseif {$count == 15} {
						set motorRollMaxpwm $buffer
						.info configure -text "TC: reading values...done"
						after 100 send_trc
					} 
					incr count 1
				} else {
					set mode ""
					set count 0
					.info configure -text "TC: error reading integer ($count): $buffer"
					.info configure -background red
				}
			} elseif {$mode == "TRC"} {
				.info configure -text "TRC: reading values...([expr $count + 1]/4): $buffer"
				if {[string is integer -strict $buffer]} {
					if {$count == 0} {
						set rcPitchMin $buffer
					} elseif {$count == 1} {
						set rcPitchMax $buffer
					} elseif {$count == 2} {
						set rcRollMin $buffer
					} elseif {$count == 3} {
						set rcRollMax $buffer
						.info configure -text "TRC: reading values...done"
						after 100 send_tac
					}
					incr count 1
				} else {
					set mode ""
					set count 0
					.info configure -text "TRC: error reading integer ($count): $buffer"
					.info configure -background red
				}
			} elseif {$mode == "TAC"} {
				.info configure -text "TAC: reading values...([expr $count + 1]/1): $buffer"
				if {[string is integer -strict $buffer]} {
					if {$count == 0} {
						set use_ACC $buffer
						.info configure -text "TAC: reading values...done"
						after 100 send_tca
					}
					incr count 1
				} else {
					set mode ""
					set count 0
					.info configure -text "TAC: error reading integer ($count): $buffer"
					.info configure -background red
				}
			} elseif {$mode == "TCA"} {
				.info configure -text "TCA: reading values...([expr $count + 1]/1): $buffer"
				if {[string is integer -strict $buffer]} {
					if {$count == 0} {
						set rcAbsolute $buffer
						.info configure -text "TCA: reading values...done"
						.info configure -background green
						.info configure -text "reading done"
						enable_all .motor
						enable_all .general.chart
						enable_all .general.settings
						enable_all .buttons
						after 100 send_trg
					}
					incr count 1
				} else {
					set mode ""
					set count 0
					.info configure -text "TCA: error reading integer ($count): $buffer"
					.info configure -background red
				}
			} elseif {$mode == "TRG"} {
				.info configure -text "TRG: reading values...([expr $count + 1]/1): $buffer"
				if {[string is integer -strict $buffer]} {
					if {$count == 0} {
						set rcGain $buffer
						.info configure -text "TRG: reading values...done"
						.info configure -background green
						.info configure -text "reading done"
						enable_all .motor
						enable_all .general.chart
						enable_all .general.settings
						enable_all .buttons
					}
					incr count 1
				} else {
					set mode ""
					set count 0
					.info configure -text "Error reading integer ($count): $buffer"
					.info configure -background red}
			} elseif {$mode == "OAC" || $mode == "ODM"} {
				.info configure -text "OAC: $buffer"
				global LastValX
				global LastValY
				set ValX [lindex $buffer 0]
				set TEST [lindex $buffer 1]
				set ValY [lindex $buffer 2]
				if {($TEST == "ACC" || $TEST == "DMP") && [string is double -strict $ValX] && [string is double -strict $ValY]} {
					incr chart_count 1
					if {$chart_count >= 450} {
						set chart_count 0
					}
					.general.chart.chart1 delete "line_$chart_count"
					.general.chart.chart1 create line $chart_count [expr $LastValX / 2 + 50] [expr $chart_count + 1] [expr $ValX / 2 + 50] -fill red -tags "line_$chart_count"
					.general.chart.chart1 create line $chart_count [expr $LastValY / 2 + 50] [expr $chart_count + 1] [expr $ValY / 2 + 50] -fill green -tags "line_$chart_count"
					.general.chart.chart1 delete "pos"
					.general.chart.chart1 create line [expr $chart_count + 1] 0 [expr $chart_count + 1] 100 -fill yellow -tags "pos"
					.general.chart.chart1 create text 5 10 -text "Pitch: $ValX" -anchor w -fill red -tags "pos"
					.general.chart.chart1 create text 5 25 -text "Roll:  $ValY" -anchor w -fill green -tags "pos"
					set LastValX $ValX
					set LastValY $ValY
				}
			} else {
				.info configure -text "INFO: $buffer"
				if {[string match "*GO!*" $buffer]} {
					.info configure -text "READY!"
					.info configure -background green
					after 100 send_tc
				}
			}
			set buffer ""
		} else {
			append buffer $ch
		}
	}
}


proc motorPitchNumber_check {n1 n2 op} {
	global motorPitchNumber
	global motorRollNumber
	catch {set motorRollNumber [expr 1 - $motorPitchNumber]}
}

proc motorRollNumber_check {n1 n2 op} {
	global motorPitchNumber
	global motorRollNumber
	catch {set motorPitchNumber [expr 1 - $motorRollNumber]}
}


label .version -text "Version: $tcl_platform(os)/$tcl_platform(osVersion)"
pack .version -side top -expand yes -fill x


frame .motor
pack .motor -side top -expand yes -fill x

	labelframe .motor.pitch -text "Pitch"
	pack .motor.pitch -side left -expand yes -fill x

		frame .motor.pitch.p
		pack .motor.pitch.p -side top -expand yes -fill x

			label .motor.pitch.p.label -text "P" -width 10
			pack .motor.pitch.p.label -side left -expand yes -fill x

			spinbox .motor.pitch.p.spin -from 0 -to 15 -increment .01 -width 10  -textvariable gyroPitchKp
			pack .motor.pitch.p.spin -side left -expand yes -fill x

		frame .motor.pitch.i
		pack .motor.pitch.i -side top -expand yes -fill x

			label .motor.pitch.i.label -text "I" -width 10
			pack .motor.pitch.i.label -side left -expand yes -fill x

			spinbox .motor.pitch.i.spin -from 0 -to 15 -increment .01 -width 10  -textvariable gyroPitchKi
			pack .motor.pitch.i.spin -side left -expand yes -fill x

		frame .motor.pitch.d
		pack .motor.pitch.d -side top -expand yes -fill x

			label .motor.pitch.d.label -text "D" -width 10
			pack .motor.pitch.d.label -side left -expand yes -fill x

			spinbox .motor.pitch.d.spin -from 0 -to 15 -increment .01 -width 10  -textvariable gyroPitchKd
			pack .motor.pitch.d.spin -side left -expand yes -fill x

		frame .motor.pitch.number
		pack .motor.pitch.number -side top -expand yes -fill x

			label .motor.pitch.number.label -text "Number" -width 10
			pack .motor.pitch.number.label -side left -expand yes -fill x

			spinbox .motor.pitch.number.spin -from 0 -to 1 -increment 1 -width 10  -textvariable motorPitchNumber
			pack .motor.pitch.number.spin -side left -expand yes -fill x

		frame .motor.pitch.dir
		pack .motor.pitch.dir -side top -expand yes -fill x

			label .motor.pitch.dir.label -text "Dir" -width 10
			pack .motor.pitch.dir.label -side left -expand yes -fill x

#			spinbox .motor.pitch.dir.spin -from 0 -to 1 -increment 1 -width 10  -textvariable motorPitchDir
#			pack .motor.pitch.dir.spin -side left -expand yes -fill x

			checkbutton .motor.pitch.dir.spin -text "reverse" -variable motorPitchDir -relief flat
			pack .motor.pitch.dir.spin -side left -expand yes -fill x


		frame .motor.pitch.poles
		pack .motor.pitch.poles -side top -expand yes -fill x

			label .motor.pitch.poles.label -text "Poles" -width 10
			pack .motor.pitch.poles.label -side left -expand yes -fill x

			spinbox .motor.pitch.poles.spin -from 3 -to 48 -increment 1 -width 10  -textvariable motorPitchPoles
			pack .motor.pitch.poles.spin -side left -expand yes -fill x

		frame .motor.pitch.maxpwm
		pack .motor.pitch.maxpwm -side top -expand yes -fill x

			label .motor.pitch.maxpwm.label -text "max PWM" -width 10
			pack .motor.pitch.maxpwm.label -side left -expand yes -fill x

			spinbox .motor.pitch.maxpwm.spin -from 0 -to 400 -increment 10 -width 10  -textvariable motorPitchMaxpwm
			pack .motor.pitch.maxpwm.spin -side left -expand yes -fill x


		frame .motor.pitch.rcmin
		pack .motor.pitch.rcmin -side top -expand yes -fill x

			label .motor.pitch.rcmin.label -text "RC-Min" -width 10
			pack .motor.pitch.rcmin.label -side left -expand yes -fill x

			spinbox .motor.pitch.rcmin.spin -from -90 -to 90 -increment 1 -width 10  -textvariable rcPitchMin
			pack .motor.pitch.rcmin.spin -side left -expand yes -fill x


		frame .motor.pitch.rcmax
		pack .motor.pitch.rcmax -side top -expand yes -fill x

			label .motor.pitch.rcmax.label -text "RC-Max" -width 10
			pack .motor.pitch.rcmax.label -side left -expand yes -fill x

			spinbox .motor.pitch.rcmax.spin -from -90 -to 90 -increment 1 -width 10  -textvariable rcPitchMax
			pack .motor.pitch.rcmax.spin -side left -expand yes -fill x



	labelframe .motor.roll -text "Roll"
	pack .motor.roll -side left -expand yes -fill x

		frame .motor.roll.p
		pack .motor.roll.p -side top -expand yes -fill x

			label .motor.roll.p.label -text "P" -width 10
			pack .motor.roll.p.label -side left -expand yes -fill x

			spinbox .motor.roll.p.spin -from 0 -to 15 -increment .01 -width 10  -textvariable gyroRollKp
			pack .motor.roll.p.spin -side left -expand yes -fill x

		frame .motor.roll.i
		pack .motor.roll.i -side top -expand yes -fill x

			label .motor.roll.i.label -text "I" -width 10
			pack .motor.roll.i.label -side left -expand yes -fill x

			spinbox .motor.roll.i.spin -from 0 -to 15 -increment .01 -width 10  -textvariable gyroRollKi
			pack .motor.roll.i.spin -side left -expand yes -fill x

		frame .motor.roll.d
		pack .motor.roll.d -side top -expand yes -fill x

			label .motor.roll.d.label -text "D" -width 10
			pack .motor.roll.d.label -side left -expand yes -fill x

			spinbox .motor.roll.d.spin -from 0 -to 15 -increment .01 -width 10  -textvariable gyroRollKd
			pack .motor.roll.d.spin -side left -expand yes -fill x

		frame .motor.roll.number
		pack .motor.roll.number -side top -expand yes -fill x

			label .motor.roll.number.label -text "Number" -width 10
			pack .motor.roll.number.label -side left -expand yes -fill x

			spinbox .motor.roll.number.spin -from 0 -to 1 -increment 10 -width 10  -textvariable motorRollNumber
			pack .motor.roll.number.spin -side left -expand yes -fill x


		frame .motor.roll.dir
		pack .motor.roll.dir -side top -expand yes -fill x

			label .motor.roll.dir.label -text "Dir" -width 10
			pack .motor.roll.dir.label -side left -expand yes -fill x

#			spinbox .motor.roll.dir.spin -from 0 -to 1 -increment 1 -width 10  -textvariable motorRollDir
#			pack .motor.roll.dir.spin -side left -expand yes -fill x

			checkbutton .motor.roll.dir.spin -text "reverse" -variable motorRollDir -relief flat
			pack .motor.roll.dir.spin -side left -expand yes -fill x


		frame .motor.roll.poles
		pack .motor.roll.poles -side top -expand yes -fill x

			label .motor.roll.poles.label -text "Poles" -width 10
			pack .motor.roll.poles.label -side left -expand yes -fill x

			spinbox .motor.roll.poles.spin -from 3 -to 48 -increment 1 -width 10  -textvariable motorRollPoles
			pack .motor.roll.poles.spin -side left -expand yes -fill x


		frame .motor.roll.maxpwm
		pack .motor.roll.maxpwm -side top -expand yes -fill x

			label .motor.roll.maxpwm.label -text "max PWM" -width 10
			pack .motor.roll.maxpwm.label -side left -expand yes -fill x

			spinbox .motor.roll.maxpwm.spin -from 0 -to 400 -increment 10 -width 10  -textvariable motorRollMaxpwm
			pack .motor.roll.maxpwm.spin -side left -expand yes -fill x


		frame .motor.roll.rcmin
		pack .motor.roll.rcmin -side top -expand yes -fill x

			label .motor.roll.rcmin.label -text "RC-Min" -width 10
			pack .motor.roll.rcmin.label -side left -expand yes -fill x

			spinbox .motor.roll.rcmin.spin -from -90 -to 90 -increment 1 -width 10  -textvariable rcRollMin
			pack .motor.roll.rcmin.spin -side left -expand yes -fill x


		frame .motor.roll.rcmax
		pack .motor.roll.rcmax -side top -expand yes -fill x

			label .motor.roll.rcmax.label -text "RC-Max" -width 10
			pack .motor.roll.rcmax.label -side left -expand yes -fill x

			spinbox .motor.roll.rcmax.spin -from -90 -to 90 -increment 1 -width 10  -textvariable rcRollMax
			pack .motor.roll.rcmax.spin -side left -expand yes -fill x


frame .general
pack .general -side top -expand yes -fill x

	labelframe .general.settings -text "General"
	pack .general.settings -side left -expand yes -fill both

		frame .general.settings.accelWeight
		pack .general.settings.accelWeight -side top -expand yes -fill x

			label .general.settings.accelWeight.label -text "Accel-Weight" -width 10
			pack .general.settings.accelWeight.label -side left -expand yes -fill x

			spinbox .general.settings.accelWeight.spin -from 0 -to 2 -increment .0001 -width 10  -textvariable accelWeight
			pack .general.settings.accelWeight.spin -side left -expand yes -fill x


		frame .general.settings.use_ACC
		pack .general.settings.use_ACC -side top -expand yes -fill x

			label .general.settings.use_ACC.label -text "ACC/DMP" -width 10
			pack .general.settings.use_ACC.label -side left -expand yes -fill x

			checkbutton .general.settings.use_ACC.check -text "use ACC" -variable use_ACC -relief flat
			pack .general.settings.use_ACC.check -side left -expand yes -fill x


		frame .general.settings.rcAbsolute
		pack .general.settings.rcAbsolute -side top -expand yes -fill x

			label .general.settings.rcAbsolute.label -text "RC Abs/Prop" -width 10
			pack .general.settings.rcAbsolute.label -side left -expand yes -fill x

			checkbutton .general.settings.rcAbsolute.check -text "Absolute" -variable rcAbsolute -relief flat
			pack .general.settings.rcAbsolute.check -side left -expand yes -fill x

		frame .general.settings.rcGain
		pack .general.settings.rcGain -side top -expand yes -fill x

			label .general.settings.rcGain.label -text "rcGain" -width 10
			pack .general.settings.rcGain.label -side left -expand yes -fill x

			spinbox .general.settings.rcGain.spin -from 0 -to 200 -increment 1 -width 10  -textvariable rcGain
			pack .general.settings.rcGain.spin -side left -expand yes -fill x

	labelframe .general.chart -text "Chart"
	pack .general.chart -side left -expand yes -fill both

		canvas .general.chart.chart1 -relief raised -width 450 -height 100
		pack .general.chart.chart1 -side left
		.general.chart.chart1 create rec 1 1 450 100 -fill black
		.general.chart.chart1 create line 0 50 450 50 -fill blue


		button .general.chart.button -text "Start" -relief raised -command {
			draw_chart
		}
		pack .general.chart.button -side left -expand yes -fill both


	labelframe .device -text "Connection"
	pack .device -side top -expand yes -fill x

		label .device.label -text "Port" -width 10
		pack .device.label -side left -expand no -fill x

		if {[catch {ttk::combobox .device.spin -textvariable device -state readonly -values $comports}]} {
			spinbox .device.spin -values $comports -width 10  -textvariable device
		}
		pack .device.spin -side left -expand yes -fill x

		button .device.connect -text "Connect" -width 9 -command {
			connect_serial
		}
		pack .device.connect -side left -expand no -fill x

		button .device.close -text "Close" -width 9 -command {
			close_serial
		}
		pack .device.close -side left -expand no -fill x


	frame .buttons
	pack .buttons -side top -expand yes -fill x

		button .buttons.defaults -text "Defaults" -width 14 -command {
			set_defaults
		}
		pack .buttons.defaults -side left -expand yes -fill x

		button .buttons.load -text "Load" -width 14 -command {
			send_tc
		}
		pack .buttons.load -side left -expand yes -fill x

		button .buttons.save -text "Save" -width 14 -command {
			save_values
		}
		pack .buttons.save -side left -expand yes -fill x

		button .buttons.gyro_cal -text "Gyro-Cal" -width 14 -command {
			gyro_cal
		}
		pack .buttons.gyro_cal -side left -expand yes -fill x

		button .buttons.load_from_flash -text "Load from Flash" -width 14 -command {
			load_from_flash
		}
		pack .buttons.load_from_flash -side left -expand yes -fill x

		button .buttons.save_to_flash -text "Save to Flash" -width 14 -command {
			save_to_flash
		}
		pack .buttons.save_to_flash -side left -expand yes -fill x


if {[string match "*Linux*" $tcl_platform(os)]} {

	frame .buttons_ext
	pack .buttons_ext -side top -expand yes -fill x

		button .buttons_ext.flash_hex_low -text "Flash Hex (Low)" -width 14 -command {
			global Serial
			catch {close $Serial}
			set Serial 0
			set file [open "/tmp/flash_hex.sh" w]
			puts $file "wget -O /tmp/_0${VERSION}_low.hex http://www.multixmedia.org/test/_0${VERSION}_low.hex"
			puts $file "avrdude -V -patmega328p -carduino -P[.device.spin get] -b57600 -D -Uflash:w:/tmp/_0${VERSION}_low.hex:i"
			close $file
			exec xterm -e sh /tmp/flash_hex.sh
		}
		pack .buttons_ext.flash_hex_low -side left -expand yes -fill x

		button .buttons_ext.flash_hex_high -text "Flash Hex (High)" -width 14 -command {
			global Serial
			catch {close $Serial}
			set Serial 0
			set file [open "/tmp/flash_hex.sh" w]
			puts $file "wget -O /tmp/_0${VERSION}_high.hex http://www.multixmedia.org/test/_0${VERSION}_high.hex"
			puts $file "avrdude -V -patmega328p -carduino -P[.device.spin get] -b57600 -D -Uflash:w:/tmp/_0${VERSION}_high.hex:i"
			close $file
			exec xterm -e sh /tmp/flash_hex.sh
		}
		pack .buttons_ext.flash_hex_high -side left -expand yes -fill x

		button .buttons_ext.reset -text "Reset" -width 14 -command {
			global Serial
			catch {close $Serial}
			set Serial 0
			set file [open "/tmp/flash_hex.sh" w]
			puts $file "avrdude -V -patmega328p -carduino -P[.device.spin get] -b57600 -D"
			close $file
			exec xterm -e sh /tmp/flash_hex.sh
		}
		pack .buttons_ext.reset -side left -expand yes -fill x
}


label .info -text "Not connected"
pack .info -side top -expand yes -fill x


trace variable motorPitchNumber w motorPitchNumber_check
trace variable motorRollNumber w motorRollNumber_check


proc disable_all {path} {
    catch {$path configure -state disabled}
    foreach child [winfo children $path] {
        disable_all $child
    }
}

proc enable_all {path} {
    catch {$path configure -state normal}
    foreach child [winfo children $path] {
        enable_all $child
    }
}

disable_all .motor
disable_all .general.chart
disable_all .general.settings
disable_all .buttons
disable_all .device.close

