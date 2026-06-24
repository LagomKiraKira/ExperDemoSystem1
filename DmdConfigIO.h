/*
 * DmdConfigIO.h
 *
 * JSON-based persistence for DMD calibration and UI configuration.
 * No external dependencies - uses standard C++ fstream.
 */

#pragma once
#ifndef DMDCONFIGIO_H_
#define DMDCONFIGIO_H_

#include <string>

struct DmdPersistConfig;

// Save DMD configuration to a JSON file
bool DmdConfig_Save(const DmdPersistConfig& cfg, const std::string& filepath);

// Load DMD configuration from a JSON file
bool DmdConfig_Load(DmdPersistConfig& cfg, const std::string& filepath);

// Get default config file path (next to executable)
std::string DmdConfig_DefaultPath();

#endif // DMDCONFIGIO_H_
