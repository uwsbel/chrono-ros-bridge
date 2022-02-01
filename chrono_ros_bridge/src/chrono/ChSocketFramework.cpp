// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/socket/ChSocketFramework.h"
#include "chrono/socket/ChExceptionSocket.h"

namespace chrono {
namespace socket {

ChSocketFramework::ChSocketFramework() {
#ifdef WINDOWS_XP

    // Initialize the winsock library
    WSADATA wsaData;

    if (WSAStartup(0x101, &wsaData)) {
        throw chrono::socket::ChExceptionSocket(0, "Error: calling WSAStartup()");
    }

#endif
}

ChSocketFramework::~ChSocketFramework() {
#ifdef WINDOWS_XP
    WSACleanup();
#endif
}

}  // namespace socket
}  // end namespace chrono
