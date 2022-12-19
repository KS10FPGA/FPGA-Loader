//******************************************************************************
//
//  KS10 Console Microcontroller
//
//! \brief
//!    FPGA Programming Utility
//!
//!
//! \file
//!    load_fpga.cpp
//!
//! \author
//!    Rob Doyle - doyle (at) cox (dot) net
//!
//! \mainpage fpga_loader
//!    The "fpga_loader" is a library that allows the KS10 executable to load
//!    FPGA firmware "on the fly" or without using the JTAG programmer and
//!    without modifying the Boot SD card.
//!
//!    The FPGA firmware file that is loaded by the "fpga_loader" library must be
//!    a valid <b>compressed</b> Raw Binary File (.rbf) - the FPGA will reject
//!    an uncompressed .rbf file.  The FPGA hardware performs
//!    the file validity checks, so an invalid (or uncompressed) .rbf file will
//!    simply fail to load without diagnostics as to the cause of the failure.
//!
//******************************************************************************
//
// Copyright (C) 2022 Rob Doyle
//
// This file is part of the KS10 FPGA Project
//
// The KS10 FPGA project is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option) any
// later version.
//
// The KS10 FPGA project is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this software.  If not, see <http://www.gnu.org/licenses/>.
//
//******************************************************************************

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>

#include "fpga_loader.hpp"

#define DEBUG(...) //printf(__VA_ARGS__)

//!
//! \brief
//!    This function loads firmware into the on-board FPGA.
//!
//! \details
//!    This program programs the FPGA from the HPS.  The procedure is
//!    described in the following document:
//!
//!    "Altera Corporation Cyclone V Device Handbook Volume 3: Hard Processor
//!    System Technical Reference Manual". See the reference below.
//!
//!    The details are in Appendix A entitled "Booting and Configuration".
//!
//!    The HPS uses the FPGA manager to configure the FPGA portion of the
//!    device. The following sequence suggests one way for software to perform
//!    a full configuration:
//!
//!    -# Set the \ref cdratio and \ref cfgwdth bits of the FPGA Manager
//!       Control Register (\ref fpgamgr_regs_t::ctrl) to match the
//!       characteristics of the configuration image. The corrects settings for
//!       \ref cdratio and \ref cfgwdth are dependant on configuration of the
//!       MSEL pins.
//!
//!    -# Set the \ref nce bit of the FPGA Manager Control Register
//!       (\ref fpgamgr_regs_t::ctrl) to 0. This will enable the HPS to modify
//!       the FPGA configuration.
//!
//!    -# Set the \ref en bit of the FPGA Manager Control Register
//!       (\ref fpgamgr_regs_t::ctrl) to 1. This will switch the FPGA
//!       configuration input signals from being controlled by pins to being
//!       controlled by the HPS.
//!
//!    -# Set the \ref nconfigpull bit of the FPGA Manager Control Register
//!       (\ref fpgamgr_regs_t::ctrl) to 1. This will assert (pull down) the
//!       nCONFIG pin and put the FPGA portion of the device into the reset
//!       state.
//!
//!    -# Poll the \ref mode bits of the FPGA Manager Control Register
//!       (\ref fpgamgr_regs_t::stat) and wait until the FPGA enters the reset
//!       state (\ref mode = \ref mode_reset).
//!
//!    -# Set the \ref nconfigpull bit of the FPGA Manager Control Register
//!       (\ref fpgamgr_regs_t::ctrl) to 0. This will negate (pull up) the
//!       nCONFIG pin and release the FPGA portion of the device from reset.
//!
//!    -# Poll the \ref mode bits of the FPGA Manager Status Register
//!       (fpgamgr_regs_t::stat) and wait until the FPGA enters the
//!        Config State (\ref mode = \ref  mode_config).
//!
//!    -# Set the \ref ns bit (nSTATUS) in the FPGA Manager GPIO EXT PORTA
//!       Register (fpgamgr_regs_t::gpio_ext_porta) to 0.
//!
//!    -# Set the \ref axicfgen bit of the FPGA Manager Control Register
//!       (\ref fpgamgr_regs_t::ctrl) to 1. This will permit the HPS to send
//!       configuration data to the FPGA.
//!
//!    -# Write the configuration data to the FPGA Manager Configuration Data
//!       register (\ref fpgamgr_data) one 32-bit word at a time until all
//!       data has been written.
//!
//!    -# Poll the FPGA Monitor Register (aka "Port A")
//!       (\ref fpgamgr_regs_gpio_ext_parta_t) to  monitor the CONF_DONE bit
//!       (\ref cd) and the nSTATUS bit (\ref ns).  Continue polling as follows:
//!
//!        -# CONF_DONE = 1 and nSTATUS = 1 (\ref cd = 1 and \ref ns = 1)
//!           indicates successful configuration.
//!
//!        -# CONF_DONE = 0 or nSTATUS = 0 (\ref cd = 0 or or \ref ns = 0)
//!           indicates unsuccessful configuration.
//!
//!        -# With any other combination except as listed above, continue
//!           polling.
//!
//!    -# Set the \ref axicfgen bit of the FPGA Manager Control Register
//!       (\ref fpgamgr_regs_t::ctrl) to 0. This will prohibit the HPS from
//!        sending configuration data to the FPGA.
//!
//!    -# Send the DCLKs required by the FPGA to enter the initialization state.
//!
//!         -# If DCLK is unused, write a value of 4 to the DCLK Count Register
//!            (\ref fpgamgr_regs_t::dclkcnt).
//!
//!         -# If DCLK is used, write a value of 20,480 (0x5000) to the DCLK
//!            Count Register (\ref fpgamgr_regs_t::dclkcnt).
//!
//!    -# Poll the \ref dcntdone bit of the DCLK Status Register
//!       (\ref fpgamgr_regs_t::dclkstat) until it changes to 1. This indicates
//!       indicates that all the DCLKs have been sent.
//!
//!    -# Write a 1 to the \ref dcntdone bit of the DCLK status register
//!       (\ref fpgamgr_regs_t::dclkstat).  This clears the completed status flag.
//!
//!    -# Poll the \ref mode bits of the  FPGA Manager Status Register
//!       (\ref fpgamgr_regs_t::stat register) and wait for FPGA to enter the
//!       User Mode state (\ref mode = \ref mode_user).
//!
//!    -# Set the \ref en bit of the FPGA Manager Control Register
//!       (\ref fpgamgr_regs_t::ctrl) to 0. This will switch the FPGA
//!       configuration input signals from being controlled by the HPS back to
//!       being controlled by the device's external pins.
//!
//! \param [in] rbf_data
//!    RBF data read from an `rbf` file.
//!
//! \param [in] rbf_size
//!    Size of the RBF file in 32-bit words.
//!
//! \param [in] debug
//!    Enables debugging messages.
//!
//! \returns
//!    <b>EXIT_FAILURE</b> if the FPGA will not transition to Reset Mode.<br>
//!    <b>EXIT_FAILURE</b> if the FPGA will not transition to Configuration Mode.<br>
//!    <b>EXIT_FAILURE</b> if the FPGA will not transition to Initialization Mode.<br>
//!    <b>EXIT_FAILURE</b> if the FPGA will not send DCLKS.<br>
//!    <b>EXIT_FAILURE</b> if the FPGA will not transition to User Mode.<br>
//!    <b>EXIT_SUCCESS</b> if the FPGA has completed programming successfully.<br>
//!
//! \pre
//!    The following preconditions must be met for programming to succeed:
//!    -# The rbf data must be 4-byte aligned, and<br>
//!    -# The size of rbf file  must be an exact multiple of 4-byte words, and<br>
//!    -# The data must be in compressed `rbf` data format, and<br>
//!    -# The MSEL[4:0] switch must be set to 0b01010. This is the default
//!       configuration for the DE10-nano.
//!
//! \note
//!    RBF is a Intel/Quartus `raw binary file`.
//!
//! \see
//!    https://www.intel.com/content/www/us/en/programmable/hps/cyclone-v/hps.html#sfo1410067849150.html
//!
//! \see
//!    https://www.intel.cn/content/dam/altera-www/global/zh_CN/pdfs/literature/hb/cyclone-v/cv_5400a.pdf
//!
//! \see
//!    https://www.intel.com/content/www/us/en/programmable/quartushelp/13.0/mergedProjects/reference/glossary/def_rbf.htm
//!

int fpga_loader_t::loadFPGA(const uint32_t *rbf_data, size_t rbf_size, bool debug) {

    //
    // mmap() the registers
    //

    int fd = open("/dev/mem", (O_RDWR | O_SYNC));

#if 0

    uint32_t       *fpgamgr_data = (uint32_t      *)mmap(NULL,      4, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, 0xffb90000);
    fpgamgr_regs_t *fpgamgr_regs = (fpgamgr_regs_t*)mmap(NULL, 0x1000, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, 0xff706000);
    sysmgr_regs_t  *sysmgr_regs  = (sysmgr_regs_t *)mmap(NULL, 0x1000, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, 0xffd08000);

    //
    // Ensure the mmap() succeeded
    //

    if (!fpgamgr_data || !fpgamgr_regs || !sysmgr_regs) {
        fprintf(stderr, "%s: unable to mmap() FPGA interface registers.\n", PROGNAME);
        exit(EXIT_FAILURE);
    }

#else

    size_t len = 0x01000000;
    char *base_addr = (char*)mmap(NULL, len, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, 0xff000000);

    //
    // Ensure the mmap() succeeded
    //

    if (!base_addr) {
        fprintf(stderr, "%s: unable to mmap() FPGA interface registers.\n", PROGNAME);
        exit(EXIT_FAILURE);
    }

    fpgamgr_regs_t *fpgamgr_regs = (fpgamgr_regs_t*)&base_addr[0x00706000];
    uint32_t       *fpgamgr_data = (uint32_t      *)&base_addr[0x00b90000];
    sysmgr_regs_t  *sysmgr_regs  = (sysmgr_regs_t *)&base_addr[0x00d08000];

#endif

#if 0

    unsigned int * regs = (unsigned int*)fpgamgr_regs;

    for (int i = 0; i < 0x20/4; i++) {
        printf("Reg [%d](0x%02x) = 0x%08x\n", i, i*4, (unsigned int)regs[i]); fflush(stdout);sleep(1);
    }


    return 0;
#endif

    //
    // Ensure that the MSEL pins are set correctly.
    //   The corrects settings for cdratio and cfgwdth are dependant on
    //   configuration of the MSEL pins.
    //

    if (get_msel(fpgamgr_regs) != 0x0a) {
        fprintf(stderr,
                "%s: "
                "The MSEL[4:0] switches need to be set to 0x0a for this program to operate properly.\n"
                "to function properly. See DE10-Nano User Manual Table 3-2.  Remember switch \"ON\"\n"
                "is a logic 0. This is the default setting of \n", PROGNAME);
    }

    DEBUG("MSEL[4:0] is correct.\n");

    //
    // Step 0.a
    //  Disable all signals from hps peripheral controller to fpga
    //

    write32(&sysmgr_regs->module, 0);

    //
    // Step 0.b
    //  Disable all signals from FPGA to HPS SDRAM
    //

#if 0
    #define SDR_CTRLGRP_FPGAPORTRST_ADDRESS     0x5080
    writel(0, SOCFPGA_SDR_ADDRESS + SDR_CTRLGRP_FPGAPORTRST_ADDRESS);
#endif

    //
    // Step 0.c:
    //  Disable all axi bridge (hps2fpga, lwhps2fpga & fpga2hps) */
    //

#if 0
    socfpga_bridges_reset(1);
#endif

    //
    // Step 1:
    //  Set the cdratio and cfgwdth bits of the FPGA Manager Control Register
    //  to match the characteristics of the configuration image.
    //

    write32(&fpgamgr_regs->ctrl, 0x01 | (read32(&fpgamgr_regs->ctrl) & 0x02c0));

    //
    // Step 2:
    //  Set the nCE bit of the FPGA Manager Control Register to 0. This will
    //  enable the HPS to modify the FPGA configuration.
    //

    write32(&fpgamgr_regs->ctrl, read32(&fpgamgr_regs->ctrl) & ~fpgamgr_regs_ctrl_t::nce);

    //
    // Step 3:
    //  Set the EN bit of the FPGA Manager Control Register to 1. This will
    //  switch the FPGA configuration input signals from being controlled by
    //  pins to being controlled by the HPS.
    //

    write32(&fpgamgr_regs->ctrl, read32(&fpgamgr_regs->ctrl) | fpgamgr_regs_ctrl_t::en);

    //
    // Step 4:
    //  Set the nCONFIG bit of the FPGA Manager Control Register  to 1. This
    //  will put the FPGA portion of the device into the reset state.
    //

    write32(&fpgamgr_regs->ctrl, read32(&fpgamgr_regs->ctrl) | fpgamgr_regs_ctrl_t::nconfigpull);

    //
    // Step 5:
    //  Poll the mode bits of the FPGA Manager Control Register nd wait until
    //  the FPGA enters the reset state.
    //

    for (int i = 0; i < 1000; i++) {
        if (get_state(fpgamgr_regs) == fpgamgr_regs_stat_t::mode_reset)
            break;
        usleep(10);
    }

    if (get_state(fpgamgr_regs) != fpgamgr_regs_stat_t::mode_reset) {
        fprintf(stderr, "%s: reset state transition failed\n", PROGNAME);
        return EXIT_FAILURE;
    }

    if (debug) {
        printf("%s: %s state\n", PROGNAME, print_state(fpgamgr_regs));
    }

    //
    // Step 6:
    //  Set the nCONFIG bit of the FPGA Manager Control Register to 0.
    //  This will release the FPGA portion of the device from reset.
    //

    write32(&fpgamgr_regs->ctrl, read32(&fpgamgr_regs->ctrl) & ~fpgamgr_regs_ctrl_t::nconfigpull);

    //
    // Step 7:
    //  Poll the mode bit of the stat register and wait until the FPGA enters
    //  the configuration state.
    //

    for (int i = 0; i < 1000; i++) {
        if (get_state(fpgamgr_regs) == fpgamgr_regs_stat_t::mode_config)
            break;
        usleep(10);
    }

    if (get_state(fpgamgr_regs) != fpgamgr_regs_stat_t::mode_config) {
        printf("%s: configuration state transition failed\n", PROGNAME);
        return EXIT_FAILURE;
    }

    if (debug) {
        printf("%s: %s state\n", PROGNAME, print_state(fpgamgr_regs));
    }

    //
    // Step 8
    //  Clear the status bits (interrupts) from the CB
    //

    write32(&fpgamgr_regs->gpio_porta_eoi, 0x00000fff);

    //
    // Step 9
    //  Set the axicfgen bit of the FPGA Manager Control Register to 1.
    //  This will permit the HPS to send configuration data to the FPGA.
    //

    write32(&fpgamgr_regs->ctrl, read32(&fpgamgr_regs->ctrl) | fpgamgr_regs_ctrl_t::axicfgen);

    //
    // Step 10
    //  Write the configuration data to the FPGA Manager Configuration Data
    //  register one 32-bit word at a time until all data has been written.
    //

    for (unsigned int i = 0; i < rbf_size; i++) {
        write32(fpgamgr_data, *rbf_data++);
    }

    //
    // Step 11
    //  Poll the FPGA Monitor Register (aka "Port A") to monitor the CONF_DONE
    //  bit and the nSTATUS bit. Continue polling as follows:
    //    a. CONF_DONE = 1 and nSTATUS = 1 indicates successful configuration.
    //    b. CONF_DONE = 0 or nSTATUS = 0 indicates unsuccessful configuration.
    //    c. With any other combination except as listed above, continue polling.
    //

    uint32_t status;
    for (int i = 0; i < 1000; i++) {
        status = read32(&fpgamgr_regs->gpio_ext_porta) & (cd | ns);
        if (status == 0) {
            printf("%s: initialization state transition failed.\n", PROGNAME);
            return EXIT_FAILURE;
        }
        if (status == (cd | ns)) {
            break;
        }
        usleep(10);
    }

    if (status != (cd | ns)) {
        printf("%s: initialization state transition failed.\n", PROGNAME);
        return EXIT_FAILURE;
    }

    if (debug) {
        printf("%s: %s state\n", PROGNAME, print_state(fpgamgr_regs));
    }

    //
    // Step 12:
    //  Set the axicfgen bit of the FPGA Manager Control Register to 0.
    //  This will prohibit the HPS from sending configuration data to the FPGA.
    //

    write32(&fpgamgr_regs->ctrl, read32(&fpgamgr_regs->ctrl) & ~fpgamgr_regs_ctrl_t::axicfgen);

    //
    // Step 13a:
    //  If the dcntdone bit of the DCLK Status Register is set, clear it.
    //

    if (read32(&fpgamgr_regs->dclkstat) != 0) {
#if 0
        write32(&fpgamgr_regs->dclkstat, 0);
#else
        write32(&fpgamgr_regs->dclkstat, 1);
#endif
    }

    //
    // Step 13b:
    //  Set the DCLK Count Register to 4. This will cause the FPGA to enter
    //  the initialization state.
    //

    write32(&fpgamgr_regs->dclkcnt, 4);

    //
    // Step 14:
    //  Poll the dcntdone bit of the DCLK status register until it
    //  changes to 1. This indicates that all the DCLKs have been sent.
    //

    for (int i = 0; i < 100; i++) {
        status = read32(&fpgamgr_regs->dclkstat) & dcntdone;
        if (status == dcntdone)
            break;
        usleep(10);
    }

    if (status != dcntdone) {
        printf("%s: time waiting for DCLKs to be sent.\n", PROGNAME);
        return EXIT_FAILURE;
    }

    //
    // Step 15:
    //  Write a 1 to the dcntdone bit of the DCLK status register to clear the
    //  completed status flag.
    //

    write32(&fpgamgr_regs->dclkstat, 1);

    //
    // Step 16
    //  Poll the mode bits of the  FPGA Manager Status Register and wait for the
    //  FPGA to enter the User Mode state.
    //

    for (int i = 0; i < 1000; i++) {
        if (get_state(fpgamgr_regs) == fpgamgr_regs_stat_t::mode_user)
            break;
        usleep(10);
    }

    if (get_state(fpgamgr_regs) != fpgamgr_regs_stat_t::mode_user) {
        printf("%s: user mode state transition failed\n", PROGNAME);
        return EXIT_FAILURE;
    }

    if (debug) {
        printf("%s: %s state\n", PROGNAME, print_state(fpgamgr_regs));
    }

    //
    // Step 17
    //  Set the EN bit of the FPGA Manager Control Register to 0. This will
    //  switch the FPGA configuration input signals from being controlled by
    //   the HPS back to being controlled by the device's external pins.
    //

    write32(&fpgamgr_regs->ctrl, read32(&fpgamgr_regs->ctrl) & ~fpgamgr_regs_ctrl_t::en);

    //
    // cleanup
    //

#if 1
    munmap(base_addr, len);
#endif

    close(fd);

    return EXIT_SUCCESS;
}
