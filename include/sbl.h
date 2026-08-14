// © 2024 Unit Circle Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.a

#pragma once
#include <stdint.h>
#include <stddef.h>

/// @brief Commands from APP to SBL
///
typedef enum sbl_cmd_e {
    SBL_CMD_NONE = 0U,   ///< Default - no command - must be 0
    SBL_CMD_INSTALL_APP, ///< Install the APP in slot 1
    SBL_CMD_RUN_APP,     ///< Run app in slot 0 - used by boot loader
} sbl_cmd_t;

/// @brief Responses from SBL to APP
///
/// Allows SBL to provide APP with indication of actions it has taken.
///
typedef enum sbl_rsp_e {
    SBL_RSP_NORMAL = 0U,       ///< Normal boot
    SBL_RSP_NEW_FW_FIRST_RUN,  ///< First run after new FW installed
    SBL_RSP_RESTORE_FIRST_RUN, ///< First run after a FW restore
    SBL_RSP_INSTALL_ERROR,     ///< Unable to install new FW - bad sig etc.
    SBL_RSP_INTERNAL_ERROR,    ///< Internal error processing command
} sbl_rsp_t;

/// @brief Starts SBL passing command
///
/// @note
/// To run SBL the function will perform a soft-reset.
///
void sbl_run(sbl_cmd_t cmd);

/// @brief Gets the SBL response
///
/// @note
/// This can only be called once after a reset as it automatically
/// clears the repsonse for next time to SBL_RSP_NORMAL.
///
/// @returns the SBL response indicating what actions (if any) it has taken.
///
sbl_rsp_t sbl_rsp(void);

/// @brief Returns the version of the SBL
///
/// E.g. 0.3.0, 2023-11-30T18:40:34Z, BFI
///
/// @returns null terminated SBL version string
///
const char *sbl_version(void);

/// @brief Returns the version string from the signature block
///
/// E.g. 0.7.0, 2023-11-31T18:40:34Z, AFI
///
/// @param p address of signature block to extract version string from
///
/// @returns null terminated APP version string
///
const char *sbl_app_version(uintptr_t p);

/// @brief Returns the image date timestamp from the signature block
///
/// @param p address of signature block to extract version string from
///
/// @returns timestamp in seconds since unix epoc.
///
uint64_t sbl_app_timestamp(uintptr_t p);

/// @brief Returns sha512 hash of the currently running APP
///
/// @returns pointer to 64 byte sha512 hash of APP
///
const uint8_t *sbl_app_hash(void);

/// @brief State structure for sha512 functions
///
typedef struct sbl_sha512_s {
    uint64_t state[32];
} sbl_sha512_t;

/// @brief Initialze state structure for computing sha512 hash
///
/// @param ctx pointer to state structure
///
void sbl_sha512_init(sbl_sha512_t *ctx);

/// @brief Provide next segment of data to be hashed
///
/// @param ctx pointer to state structure
/// @param buffer pointer to data to hash
/// @param number of bytes in buffer to hash
///
void sbl_sha512_update(sbl_sha512_t *ctx, const void *buffer, size_t len);

/// @brief Output the sha512 hash result
///
/// @param ctx pointer to state structure
/// @param hash pointer to where to store hash - must be at least 64 bytes.
///
void sbl_sha512_final(sbl_sha512_t *ctx, void *hash);
