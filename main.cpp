//******************************************************************************
//
//  KS10 Console Microcontroller
//
//! \brief
//!    FPGA Programming Utility
//!
//! \file
//!    main.cpp
//!
//! \author
//!    Rob Doyle - doyle (at) cox (dot) net
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <getopt.h>

#include "fpga_loader.hpp"

//!
//! \brief
//!    This function loads firmware into the on-board FPGA.
//!
//! \param[in] argc
//!    argc is the number of arguments provided.
//!
//! \param[in] argv
//!    argv is an array of arguments.
//!
//! \returns
//!    EXIT_SUCCESS or EXIT_FAILURE
//!

int main(int argc, char *argv[]) {

    const char *usage =
        "\n"
        "The fpga_loader is an executable for the DE10-Nano that allows the target\n"
        "device to load its own FPGA firmware.\n"
        "\n"
        "usage: " PROGNAME " [options] \"raw_binary_file.rbf\"\n"
        "\n"
        "Valid options are:\n"
        "  --debug         Print debug messages.\n"
        "  --help          Print help message and exit.\n"
        "  --quiet         Suppress messages.\n"
        "\n"
        "Note: The FPGA firmware must be in Raw Binary File (RBF) format.\n"
        "\n";

    //
    // Sort command line
    //

    static const struct option options[] = {
        {"help",   no_argument,       0, 0},  // 0
        {"debug",  no_argument,       0, 0},  // 1
        {"q",      no_argument,       0, 0},  // 2
        {"quiet",  no_argument,       0, 0},  // 3
        {0,        0,                 0, 0},  // 4
    };

    int index = 0;
    bool debug = false;
    bool quiet = false;
    opterr = 0;
    for (;;) {
        int ret = getopt_long(argc, argv, "", options, &index);
        if (ret == -1) {
            break;
        } else if (ret == '?') {
            printf("%s: unrecognized option: %s\n", PROGNAME, argv[optind-1]);
            printf(usage);
            return true;
        } else {
            switch(index) {
                case 0:
                    printf(usage);
                    return true;
                case 1:
                    debug = true;
                    break;
                case 2:
                case 3:
                    quiet = true;
                    break;
            }
        }
    }

    //
    // Check that the program arguments are correct
    //

    if (argv[optind] == NULL) {
        printf("%s: missing filename\n", PROGNAME);
        printf(usage);
        return EXIT_FAILURE;
    }

    //
    // Open the firmware file
    //

    FILE *fd = fopen(argv[optind], "r");
    if (!fd) {
        perror(PROGNAME);
        return EXIT_FAILURE;
    }

    //
    // Read the rbf file
    //

    uint32_t *buf = (uint32_t*)malloc(8*1024*1024);
    size_t size = fread(buf, 1, 8*1024*1024, fd);
    if (size == 0) {
        perror(PROGNAME);
        return EXIT_FAILURE;
    }

    if (!quiet) {
        printf("%s: Successfully read file \"%s\" (%d bytes).\n", PROGNAME, argv[optind], size);
    }

    //
    // Check input buffer alignment
    //

    uint32_t addr = (uint32_t)buf;
    if ((addr & 0x03) != 0) {
        fprintf(stderr, "%s: rbf data buffer is not aligned properly.\n", PROGNAME);
        exit(EXIT_FAILURE);
    }

    //
    // Check file length alignment
    //

    if ((size & 0x03) != 0) {
        fprintf(stderr, "%s: rbf file length is not exact multiple of 32-bit words.\n", PROGNAME);
        exit(EXIT_FAILURE);
    }

    //
    // size is the number of 32-bit words
    //

    size /= sizeof(uint32_t);

    //
    // Program the FPGA
    //

    fpga_loader_t fpga_loader;
    int ret = fpga_loader.loadFPGA(buf, size, debug);
    free(buf);

    //
    // Cleanup
    //

    if (!quiet && (ret == EXIT_SUCCESS)) {
        printf("%s: FPGA progammed successfully\n", PROGNAME);
    }

    return ret;
}
