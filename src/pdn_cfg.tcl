# Copyright 2025 LibreLane Contributors
#
# Adapted from OpenLane
#
# Copyright 2020-2022 Efabless Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ---------------------------------------------------------------------------
# Loom (tt_um_loom, branch sram-smoke) changes, SPDX-License-Identifier: Apache-2.0
# https://github.com/thomasgilbert481/tt_um_loom at 7daa439c
#
# Changed for this project: the helpers are renamed and the comments rewritten.
# ---------------------------------------------------------------------------
#
# PDN for the RM_IHPSG13_1P_512x16_c2_bm_bist macro on sg13cmos5l.
#
# Below this comment is LibreLane 3.1.0.dev3's scripts/openroad/common/pdn_cfg.tcl
# with one change: its macro grid, the last ten lines, is replaced by the pdngen
# wrapper at the end of this file. Stripe geometry comes from the FP_PDN_* keys
# in src/config.json.
#
# Why the default macro grid cannot work here. It connects a macro to the block
# grid with a Metal4 to TopMetal1 via, but a block has no TopMetal1: that layer
# belongs to the chip top and precheck forbids it. The macro's power pins are on
# Metal4, the block's only stripe layer, so a same-layer overlap is the only
# possible connection.
#
# pdngen will not make that overlap. It treats a FIXED macro's Metal4 as an
# obstruction and cuts each stripe short of it, which leaves the macro floating.
# Bridging the cut ends connects the macro, but precheck wants every Metal4 power
# port to reach within 10 um of both the top and the bottom edge, so a cut stripe
# fails there.
#
# So the stripes run the full height, through the macro's power columns. pdngen
# only makes obstructions of FIXED instances, so the wrapper marks the macros
# PLACED while pdngen builds its shapes and restores them before anything is
# written. Then it checks that every stripe over a macro lies inside power
# columns of its own net, and that every macro supply pin has a stripe, and it
# runs check_power_grid on every supply. Any failure stops the step.
#
# The one gap. A regular POWER column carries VDD! up to local y 38.825 and
# VDDARRAY! from 45.465, with a Metal4 OBS across it in between. The POWER
# stripes cross that band and join the two, which PDN_MACRO_CONNECTIONS ties to
# VPWR anyway.
#
# The stripe numbers, from the macro's LEF. Columns are 2.81 um wide, macro-local:
#   VDD! / VDDARRAY!  x0 = 4.26 + 11.24 k  and 151.05 + 11.24 k  (k = 0..7)
#   VSS!              x0 = 9.88 + 11.24 k  and 145.43 + 11.24 k  (k = 0..7)
#   irregular middle band from x 88 to x 151
#   VWIDTH   2.1    Precheck's minimum power port width.
#   VSPACING 3.52   The GROUND stripe sits (spacing + width) = 5.62 um from POWER,
#                   the macro's POWER to GROUND column distance.
#   VPITCH   67.44  6 x 11.24, the smallest multiple that steps over the middle band.
#   VOFFSET  26.36  From the core xMin of 2.88: 2.88 + 26.36 = 29.24 = 12.0 (macro x)
#                   + 17.24, which splits the 0.67 um offset between the macro's two
#                   halves, so both halves' stripes sit inside their columns.
# The macro's x must stay 12 + 67.44 k, or VOFFSET must be recomputed. Do not add
# -snap_to_grid: Metal4 tracks do not line up with the columns.

source $::env(SCRIPTS_DIR)/openroad/common/io.tcl
source $::env(SCRIPTS_DIR)/openroad/common/set_global_connections.tcl
set_global_connections

set secondary []
foreach vdd $::env(VDD_NETS) gnd $::env(GND_NETS) {
    if { $vdd != $::env(VDD_NET)} {
        lappend secondary $vdd

        set db_net [[ord::get_db_block] findNet $vdd]
        if {$db_net == "NULL"} {
            set net [odb::dbNet_create [ord::get_db_block] $vdd]
            $net setSpecial
            $net setSigType "POWER"
        }
    }

    if { $gnd != $::env(GND_NET)} {
        lappend secondary $gnd

        set db_net [[ord::get_db_block] findNet $gnd]
        if {$db_net == "NULL"} {
            set net [odb::dbNet_create [ord::get_db_block] $gnd]
            $net setSpecial
            $net setSigType "GROUND"
        }
    }
}

set_voltage_domain -name CORE -power $::env(VDD_NET) -ground $::env(GND_NET) \
    -secondary_power $secondary



if { $::env(PDN_MULTILAYER) == 1 } {

    set arg_list [list]
    if { $::env(PDN_ENABLE_PINS) } {
        lappend arg_list -pins "$::env(PDN_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"
    }

    define_pdn_grid \
        -name stdcell_grid \
        -starts_with POWER \
        -voltage_domain CORE \
        {*}$arg_list

    set arg_list [list]
    append_if_equals arg_list PDN_EXTEND_TO "core_ring" -extend_to_core_ring
    append_if_equals arg_list PDN_EXTEND_TO "boundary" -extend_to_boundary

    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(PDN_VERTICAL_LAYER) \
        -width $::env(PDN_VWIDTH) \
        -pitch $::env(PDN_VPITCH) \
        -offset $::env(PDN_VOFFSET) \
        -spacing $::env(PDN_VSPACING) \
        -starts_with POWER \
        {*}$arg_list

    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(PDN_HORIZONTAL_LAYER) \
        -width $::env(PDN_HWIDTH) \
        -pitch $::env(PDN_HPITCH) \
        -offset $::env(PDN_HOFFSET) \
        -spacing $::env(PDN_HSPACING) \
        -starts_with POWER \
        {*}$arg_list

    add_pdn_connect \
        -grid stdcell_grid \
        -layers "$::env(PDN_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"
} else {

    set arg_list [list]
    if { $::env(PDN_ENABLE_PINS) } {
        lappend arg_list -pins "$::env(PDN_VERTICAL_LAYER)"
    }

    define_pdn_grid \
        -name stdcell_grid \
        -starts_with POWER \
        -voltage_domain CORE \
        {*}$arg_list

    set arg_list [list]
    append_if_equals arg_list PDN_EXTEND_TO "core_ring" -extend_to_core_ring
    append_if_equals arg_list PDN_EXTEND_TO "boundary" -extend_to_boundary

    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(PDN_VERTICAL_LAYER) \
        -width $::env(PDN_VWIDTH) \
        -pitch $::env(PDN_VPITCH) \
        -offset $::env(PDN_VOFFSET) \
        -spacing $::env(PDN_VSPACING) \
        -starts_with POWER \
        {*}$arg_list
}

# Adds the standard cell rails if enabled.
if { $::env(PDN_ENABLE_RAILS) == 1 } {
    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(PDN_RAIL_LAYER) \
        -width $::env(PDN_RAIL_WIDTH) \
        -followpins

    add_pdn_connect \
        -grid stdcell_grid \
        -layers "$::env(PDN_RAIL_LAYER) $::env(PDN_VERTICAL_LAYER)"
}


# Adds the core ring if enabled.
if { $::env(PDN_CORE_RING) == 1 } {
    if { $::env(PDN_MULTILAYER) == 1 } {
        set arg_list [list]
        append_if_flag arg_list PDN_CORE_RING_ALLOW_OUT_OF_DIE -allow_out_of_die
        append_if_flag arg_list PDN_CORE_RING_CONNECT_TO_PADS -connect_to_pads
        append_if_equals arg_list PDN_EXTEND_TO "boundary" -extend_to_boundary
        append_if_exists_argument arg_list PDN_CORE_RING_CONNECT_TO_PAD_LAYERS -connect_to_pad_layers

        set pdn_core_vertical_layer $::env(PDN_VERTICAL_LAYER)
        set pdn_core_horizontal_layer $::env(PDN_HORIZONTAL_LAYER)

        if { [info exists ::env(PDN_CORE_VERTICAL_LAYER)] } {
            set pdn_core_vertical_layer $::env(PDN_CORE_VERTICAL_LAYER)
        }

        if { [info exists ::env(PDN_CORE_HORIZONTAL_LAYER)] } {
            set pdn_core_horizontal_layer $::env(PDN_CORE_HORIZONTAL_LAYER)
        }

        add_pdn_ring \
            -grid stdcell_grid \
            -layers "$pdn_core_vertical_layer $pdn_core_horizontal_layer" \
            -widths "$::env(PDN_CORE_RING_VWIDTH) $::env(PDN_CORE_RING_HWIDTH)" \
            -spacings "$::env(PDN_CORE_RING_VSPACING) $::env(PDN_CORE_RING_HSPACING)" \
            -core_offset "$::env(PDN_CORE_RING_VOFFSET) $::env(PDN_CORE_RING_HOFFSET)" \
            {*}$arg_list

        if { [info exists ::env(PDN_CORE_VERTICAL_LAYER)] } {
            add_pdn_connect \
                -grid stdcell_grid \
                -layers "$::env(PDN_CORE_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"
        }

        if { [info exists ::env(PDN_CORE_HORIZONTAL_LAYER)] } {
            add_pdn_connect \
                -grid stdcell_grid \
                -layers "$::env(PDN_CORE_HORIZONTAL_LAYER) $::env(PDN_VERTICAL_LAYER)"
        }

        if { [info exists ::env(PDN_CORE_VERTICAL_LAYER)] && [info exists ::env(PDN_CORE_HORIZONTAL_LAYER)] } {
            add_pdn_connect \
                -grid stdcell_grid \
                -layers "$::env(PDN_CORE_VERTICAL_LAYER) $::env(PDN_CORE_HORIZONTAL_LAYER)"
        }

    } else {
        throw APPLICATION "PDN_CORE_RING cannot be used when PDN_MULTILAYER is set to false."
    }
}


# Replaces the default macro grid: full-height stripes through the macro's power
# columns, then a check of the result. See the header.

# Longest stretch of a macro that a stripe may cross without a same-net pin under
# it, and only between two same-net pins. The VDD!/VDDARRAY! split is 6.64 um.
set ::pe_max_pin_gap 7.0

proc pe_check_macro_stripes {} {
    set block [ord::get_db_block]
    set layer_name $::env(PDN_VERTICAL_LAYER)
    set dbu [$block getDbUnitsPerMicron]
    set max_gap [expr {round($::pe_max_pin_gap * $dbu)}]
    set um [expr {1.0 / $dbu}]

    # 1. Every power and ground pin rectangle of every hard macro on the stripe
    #    layer, in die coordinates, with the block net it is tied to.
    set macros [list]
    set columns [list]
    set counts [dict create]
    foreach inst [$block getInsts] {
        if { ![[$inst getMaster] isBlock] } { continue }
        set iname [$inst getName]
        set orient [$inst getOrient]
        if { $orient ne "R0" && $orient ne "MX" } {
            error "PEPDN: $iname has orientation $orient; only R0 (N) and MX (FS) keep the Metal4 power columns vertical"
        }
        set bb [$inst getBBox]
        set bx0 [$bb xMin]
        set by0 [$bb yMin]
        set bx1 [$bb xMax]
        set by1 [$bb yMax]
        lappend macros [list $iname $bx0 $by0 $bx1 $by1]
        puts [format "PEPDN macro %s orient %s bbox %.3f %.3f %.3f %.3f status %s" $iname $orient \
            [expr {$bx0 * $um}] [expr {$by0 * $um}] [expr {$bx1 * $um}] [expr {$by1 * $um}] \
            [$inst getPlacementStatus]]
        foreach iterm [$inst getITerms] {
            set mterm [$iterm getMTerm]
            set sig [$mterm getSigType]
            if { $sig ne "POWER" && $sig ne "GROUND" } { continue }
            set key "$iname/[$mterm getName]"
            set net [$iterm getNet]
            if { $net eq "NULL" } {
                error "PEPDN: $key is not connected to any net (check PDN_MACRO_CONNECTIONS)"
            }
            dict set counts $key 0
            set ux1 ""
            foreach mpin [$mterm getMPins] {
                foreach box [$mpin getGeometry] {
                    if { [$box isVia] } { continue }
                    if { [[$box getTechLayer] getName] ne $layer_name } { continue }
                    set x1 [expr {$bx0 + [$box xMin]}]
                    set x2 [expr {$bx0 + [$box xMax]}]
                    if { $orient eq "R0" } {
                        set y1 [expr {$by0 + [$box yMin]}]
                        set y2 [expr {$by0 + [$box yMax]}]
                    } else {
                        set y1 [expr {$by1 - [$box yMax]}]
                        set y2 [expr {$by1 - [$box yMin]}]
                    }
                    lappend columns [list $net $key $x1 $y1 $x2 $y2]
                    if { $ux1 eq "" } {
                        lassign [list $x1 $y1 $x2 $y2] ux1 uy1 ux2 uy2
                    } else {
                        set ux1 [expr {min($ux1, $x1)}]
                        set uy1 [expr {min($uy1, $y1)}]
                        set ux2 [expr {max($ux2, $x2)}]
                        set uy2 [expr {max($uy2, $y2)}]
                    }
                }
            }
            # Cross-check the transform above against OpenDB's own.
            set ib [$iterm getBBox]
            if { $ux1 ne "" && ($ux1 != [$ib xMin] || $uy1 != [$ib yMin] || $ux2 != [$ib xMax] || $uy2 != [$ib yMax]) } {
                error "PEPDN: transformed pins of $key ($ux1 $uy1 $ux2 $uy2) disagree with the ITerm bbox ([$ib xMin] [$ib yMin] [$ib xMax] [$ib yMax])"
            }
        }
    }
    if { [llength $macros] == 0 } {
        puts "PEPDN no hard macros; nothing to check"
        return
    }

    # 2. Every stripe-layer shape of every net, against every macro.
    set bad [list]
    foreach net [$block getNets] {
        if { ![$net isSpecial] } { continue }
        foreach swire [$net getSWires] {
            foreach sbox [$swire getWires] {
                if { [$sbox isVia] } { continue }
                if { [[$sbox getTechLayer] getName] ne $layer_name } { continue }
                set sx1 [$sbox xMin]
                set sy1 [$sbox yMin]
                set sx2 [$sbox xMax]
                set sy2 [$sbox yMax]
                foreach m $macros {
                    lassign $m iname mx0 my0 mx1 my1
                    if { $sx2 <= $mx0 || $sx1 >= $mx1 || $sy2 <= $my0 || $sy1 >= $my1 } { continue }
                    set desc [format "%s x %.3f..%.3f y %.3f..%.3f" [$net getName] \
                        [expr {$sx1 * $um}] [expr {$sx2 * $um}] [expr {$sy1 * $um}] [expr {$sy2 * $um}]]
                    set ylo [expr {max($sy1, $my0)}]
                    set yhi [expr {min($sy2, $my1)}]
                    # Same-net pin rectangles that contain the stripe's x range.
                    set cover [list]
                    foreach c $columns {
                        lassign $c cnet key px1 py1 px2 py2
                        if { $cnet ne $net } { continue }
                        if { $px1 > $sx1 || $px2 < $sx2 } { continue }
                        if { $py2 <= $ylo || $py1 >= $yhi } { continue }
                        lappend cover [list $py1 $py2 $key]
                    }
                    set cover [lsort -integer -index 0 $cover]
                    set cur $ylo
                    set keys [list]
                    set why ""
                    foreach r $cover {
                        lassign $r py1 py2 key
                        if { $py1 > $cur } {
                            if { $cur == $ylo } {
                                set why [format "no same-net pin under it from y %.3f to %.3f" [expr {$cur * $um}] [expr {$py1 * $um}]]
                                break
                            }
                            if { $py1 - $cur > $max_gap } {
                                set why [format "crosses %.3f um without a same-net pin (y %.3f..%.3f)" \
                                    [expr {($py1 - $cur) * $um}] [expr {$cur * $um}] [expr {$py1 * $um}]]
                                break
                            }
                            puts [format "PEPDN   %s crosses a %.3f um gap between same-net pins at y %.3f..%.3f" \
                                $desc [expr {($py1 - $cur) * $um}] [expr {$cur * $um}] [expr {$py1 * $um}]]
                        }
                        set cur [expr {max($cur, $py2)}]
                        lappend keys $key
                    }
                    if { $why eq "" && $cur < $yhi } {
                        set why [format "no same-net pin under it from y %.3f to %.3f" [expr {$cur * $um}] [expr {$yhi * $um}]]
                    }
                    if { $why ne "" } {
                        lappend bad "$desc over $iname: $why"
                        continue
                    }
                    foreach key [lsort -unique $keys] { dict incr counts $key }
                    puts "PEPDN stripe $desc runs inside [join [lsort -unique $keys] { + }]"
                }
            }
        }
    }
    if { [llength $bad] > 0 } {
        foreach b $bad { puts "PEPDN BAD $b" }
        error "PEPDN: [llength $bad] stripe shape(s) over a macro are not inside same-net power pins (would short or float); fix FP_PDN_V* or the macro location"
    }

    # 3. Every supply pin of every macro needs at least one stripe.
    set missing [list]
    dict for {key n} $counts {
        puts "PEPDN $key: $n stripe(s)"
        if { $n == 0 } { lappend missing $key }
    }
    if { [llength $missing] > 0 } {
        error "PEPDN: no stripe runs through: [join $missing {, }]"
    }
}

# Wraps LibreLane's one pdngen call. OpenROAD's pdngen parses its flags and then
# runs pdn::check_setup, pdn::build_grids, pdn::write_to_db and pdn::reset_shapes.
# check_setup wants every macro placed and fixed, while build_grids makes
# obstructions of fixed instances. So the wrapper runs those four steps itself and
# releases the macros around build_grids only. Any other flag goes to the original.
if { [info commands ::pe_pdngen_unwrapped] eq "" } {
    rename ::pdngen ::pe_pdngen_unwrapped
    proc ::pdngen { args } {
        foreach a $args {
            if { $a ne "-skip_trim" } {
                return [::pe_pdngen_unwrapped {*}$args]
            }
        }
        foreach cmd {pdn::check_setup pdn::build_grids pdn::write_to_db pdn::reset_shapes} {
            if { [info commands ::$cmd] eq "" } {
                error "PEPDN: $cmd does not exist in this OpenROAD"
            }
        }
        set trim [expr {[lsearch -exact $args -skip_trim] < 0}]

        ::pdn::check_setup

        set released [list]
        foreach inst [[ord::get_db_block] getInsts] {
            if { [[$inst getMaster] isBlock] && [$inst isFixed] } {
                lappend released [list $inst [$inst getPlacementStatus]]
                $inst setPlacementStatus "PLACED"
            }
        }
        set rc [catch { ::pdn::build_grids $trim } msg opts]
        foreach r $released {
            lassign $r inst status
            $inst setPlacementStatus $status
        }
        if { $rc } {
            return -options $opts $msg
        }

        ::pdn::write_to_db 1 ""
        ::pdn::reset_shapes

        pe_check_macro_stripes
        foreach net_name [concat $::env(VDD_NETS) $::env(GND_NETS)] {
            puts "PEPDN check_power_grid -net $net_name"
            check_power_grid -net $net_name
        }
    }
}
