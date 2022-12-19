//******************************************************************************
//
//  KS10 Console Microcontroller
//
//! \brief
//!    FPGA programmer utility header file
//!
//! \details
//!    This object allows the KS10 Console (HPS) to program the FPGA firmware.
//!
//! \file
//!    fpga_loader.hpp
//!
//! \author
//!    Rob Doyle - doyle (at) cox (dot) net
//
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
//

#ifndef __FPGA_LOADER_H
#define __FPGA_LOADER_H

#include <stdint.h>

#define PROGNAME "fpga_loader"

//!
//! \brief
//!    The fpgamgr registers
//!

struct fpgamgr_regs_t {
    uint32_t stat;                              //!< (0x000) FPGA Manager Status Register
    uint32_t ctrl;                              //!< (0x004) FPGA Manager Control Register
    uint32_t dclkcnt;                           //!< (0x008) Register which allows software to send DCLKS to the FPGA during configuation.
    uint32_t dclkstat;                          //!< (0x00c) Register which reports status of the DCLK counter (DCLKCNT)
    uint32_t gpo;                               //!< (0x010) General purpose outputs to FPGA fabric
    uint32_t gpi;                               //!< (0x014) General purpose input from FPGA fabric
    uint32_t misci;                             //!< (0x018) FPGA boot status
    uint32_t pad1[(0x830-0x1c)/4];              //!< (0x01c-0x830) Address space padding
    uint32_t gpio_inten;                        //!< (0x830) Interrupt enables for Port A
    uint32_t gpio_intmask;                      //!< (0x834) Interrupt masks for Port A
    uint32_t gpio_inttype_level;                //!< (0x838) Interrupt type for Port A
    uint32_t gpio_int_polarity;                 //!< (0x83c) Interrupt polarity for Port A
    uint32_t gpio_intstatus;                    //!< (0x840) Interrupt status for Port A
    uint32_t gpio_raw_intstatus;                //!< (0x844) Interrupt status (raw) for Port A
    uint32_t pad2;                              //!< (0x848) Address space padding
    uint32_t gpio_porta_eoi;                    //!< (0x84c) End-of-interrupt for Port A
    uint32_t gpio_ext_porta;                    //!< (0x850) GPIO interface to Port A
    uint32_t pad3;                              //!< (0x854) Address space padding
    uint32_t pad4;                              //!< (0x858) Address space padding
    uint32_t pad5;                              //!< (0x85c) Address space padding
    uint32_t gpio_1s_sync;                      //!< (0x860) GPIO syncronizatoin
    uint32_t pad6;                              //!< (0x864) Address space padding
    uint32_t pad7;                              //!< (0x868) Address space padding
    uint32_t gpio_ver_id_code;                  //!< (0x86c) GPIO Component Version
    uint32_t gpio_config_reg2;                  //!< (0x870) Specifies the bit width of Port A
    uint32_t gpio_config_reg1;                  //!< (0x874) Reports settings of various GPIO configuration parameters
};

//!<
//!< \brief
//!<    Register defintion of the SYSMGR
//!<

struct sysmgr_regs_t {
    uint32_t siliconid1;                        //!< (0x000) Silicon ID and revision number
    uint32_t siliconid2;                        //!< (0x004) Reserved for future use
    uint32_t pad1;                              //!< (0x008) Address space padding
    uint32_t pad2;                              //!< (0x00c) Address space padding
    uint32_t wddbg;                             //!< (0x010) Controls the behavior of the L4 watchdogs when the CPUs are in debug mode
    uint32_t bootinfo;                          //!< (0x014) Provides access to boot configuration information
    uint32_t hpsinfo;                           //!< (0x018) Provides information about the HPS capabilities
    uint32_t parityinj;                         //!< (0x01c) Allow parity circuitry to be tested by injecting parity failures into the parity-protected RAMs in the MPU
    uint32_t gbl;                               //!< (0x020) Used to enable/disable ALL interfaces between the FPGA and HPS
    uint32_t indiv;                             //!< (0x024) Used to enable/disable selected interfaces between the FPGA and HPS
    uint32_t module;                            //!< (0x028) Used to enable/disable signals from the FPGA fabric to individual HPS modules.
    uint32_t pad3;                              //!< (0x030) Address space padding
};

//!
//! \brief
//!    FPGA Loader object
//!

class fpga_loader_t  {

    private:

        //!
        //! \brief
        //!    Bit definitions of the FPGAMGR Control register
        //!

        enum fpgamgr_regs_ctrl_t : uint32_t {
            en           = 0x00000001,          //!< Asserted to permit HPS to drive configuration inputs to the CB.
            nce          = 0x00000002,          //!< Asserted to negate the nCE (chip enable) input to the CB.
            nconfigpull  = 0x00000004,          //!< Asserted to negate the nCONFIG input to the CB.
            nstatuspull  = 0x00000008,          //!< Asserted to negate the nSTATUS input to the CB.
            confdonepull = 0x00000010,          //!< Asserted to negate the CONF_DONE input to the CB.
            prreq        = 0x00000020,          //!< Asserted when requesting partial reconfiguration.
            cdratio      = 0x000000c0,          //!< Clock to Data Ratio (CDRATIO) for configuration data transfer from the AXI Slave to the FPGA (MSB).
            axicfgen     = 0x00000100,          //!< Asserted to enable DCLK during the AXI configuration data transfers.
            cfgwdth      = 0x00000200,          //!< Configuration Passive Parallel data bus width.
        };

        //!
        //! \brief
        //!    Bit definitions of the FPGAMGR Status register
        //!

        enum fpgamgr_regs_stat_t : uint32_t {
            mode_reset   = 0x00000001,          //!< FPGA in Reset state
            mode_config  = 0x00000002,          //!< FPGA in Configuration state
            mode_init    = 0x00000003,          //!< FPGA in Initialization state
            mode_user    = 0x00000004,          //!< FPGA in User Mode state
            mode         = 0x00000007,          //!< FPGA mode bits
        };

        //!
        //! \brief
        //!    Bit definitions of the FPGAMGR GPIO EXT PORTA register
        //!

        enum fpgamgr_regs_gpio_ext_parta_t : uint32_t {
            ns           = 0x00000001,          //!< Reports value of the nSTATUS signal
            cd           = 0x00000002,          //!< Reports value of the CONF_DONE signal
            id           = 0x00000004,          //!< Reports value of the INIT_DONE signal
            crc          = 0x00000008,          //!< Reports value of the CRC_ERROR signal
            ccd          = 0x00000010,          //!< Reports value of the CVP_CONF_DONE signal
            prr          = 0x00000020,          //!< Reports value of the PR_READY signal
            pre          = 0x00000040,          //!< Reports value of the PR_ERROR signal
            prd          = 0x00000080,          //!< Reports value of the PR_DONE signal
            ncp          = 0x00000100,          //!< Reports value of the nCONFIG pin
            nsp          = 0x00000200,          //!< Reports value of the nSTATUS pin
            cdp          = 0x00000400,          //!< Reports value of the CONF_DONE pin
            fpo          = 0x00000800,          //!< Reports value of the FPGA_POWER_ON signal
        };

        //!
        //! \brief
        //!    Bit definitions of the FPGAMGR DCLK Status register
        //!

        enum fpgamgr_regs_dclkstat_t : uint32_t {
            dcntdone     = 0x00000001,          //!< Asserted when DCLKCNT has decremented to zero
        };

        //!
        //! \brief
        //!    Read a 32-bit word from IO
        //!
        //! \param[in] addr
        //!    IO register address
        //!
        //! \note
        //!    This is native endian.
        //!

        uint32_t read32(volatile void *addr) {
            return *(uint32_t*)addr;
        }

        //!
        //! \brief
        //!    Write a 32-bit word to IO
        //!
        //! \param[in] addr
        //!    IO register address
        //!
        //! \param[in] val
        //!    Data to be written to the IO location
        //!
        //! \note
        //!    This is native endian.
        //!

        void write32(volatile void *addr, uint32_t val) {
            *(uint32_t*)addr = val;
        }

        //!
        //! \brief
        //!    Get the state of the MSEL[4:0] pins.
        //!
        //! \param[in] addr
        //!    FPGA Manager base address.
        //!
        //! \returns
        //!    The contents of the MSEL[4:0] bits in the FPGAMGR Status Register
        //!
        //! \note
        //!    This function adds the status register offset to the base address
        //!    parameter.
        //!

        uint32_t get_msel(fpgamgr_regs_t *addr) {
            return (read32(&addr->stat) >> 3) & 0x1f;
        }

        //!
        //! \brief
        //!    Get the state of the FPGA configuration.
        //!
        //! \param[in] addr
        //!    FPGA Manager base address.
        //!
        //! \returns
        //!    The contents of the mode[2:0] bits in the FPGAMGR Status Register
        //!
        //! \note
        //!    This function adds the status register offset to the base address
        //!    parameter.
        //!

        uint32_t get_state(fpgamgr_regs_t *addr) {
            return (read32(&addr->stat) >> 0) & 0x07;
        }

        //!
        //! \brief
        //!    This function reads the FPGAMGR Status Register and prints the
        //!    the FPGA state.
        //!
        //! \param[in] addr
        //!    Pointer to the base address of the FPGAMGR registers.
        //!
        //! \returns
        //!    Pointer to a string that contains the state.
        //!

        const char *print_state(fpgamgr_regs_t *addr) {
            switch (get_state(addr)) {
                case  0: return "Off";
                case  1: return "Reset";
                case  2: return "Configuration";
                case  3: return "Initialization";
                case  4: return "User";
                default: return "Undetermined";
            }
        }

    public:

        int loadFPGA(const uint32_t *rbf_data, size_t rbf_size, bool debug);

};

#endif
