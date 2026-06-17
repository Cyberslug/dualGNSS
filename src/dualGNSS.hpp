#pragma once

#include "UbxGNSS.hpp"
#include "CasicGNSS.hpp"
#include "GnssTypes.hpp"   // GnssType enum, UbxSeries enum

/**
* @brief  Forward declaration of the unified GNSS facade.
*         Users instantiate Gnss<GnssType::UBX, UbxSeries::xxxxx>
*         or Gnss<GnssType::CASIC>.  The implementation classes
*         UbxGNSS and CasicGNSS are library-internal detail.
*/
template<GnssType TType, UbxSeries TSeries = UbxSeries::UNKNOWN>
class Gnss;


/**
* @brief  UBX specialisation.
*         TProtocol selects the UBX wire protocol version at compile time.
*         UbxGNSS is inherited privately so its API is not exposed directly.
*/
template<UbxSeries TSeries>
class Gnss<GnssType::UBX, TSeries> : private UbxGNSS
{
  public:
    Gnss() : UbxGNSS(TSeries) {}

    bool  begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
                { return UbxGNSS::begin(serial, rxPin, txPin);       }
    void  beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                    uint32_t baud = UbxConfigurator::TARGET_BAUD_RATE)
                    {UbxGNSS::beginPassive(serial, rxPin, txPin, baud);}
    void  update()       { UbxGNSS::update();      }
    bool  hasNewData()   { return UbxGNSS::hasNewData();  }
    bool  isFixValid()   { return UbxGNSS::isFixValid();  }
    void  getData(GnssData& dest)   { UbxGNSS::getData(dest);  }
    GnssConfigResult getConfigResult();
};

/**
* @brief  UBX specialisation.
*         ConfigurationResult translation
*/
template<UbxSeries TSeries>
GnssConfigResult Gnss<GnssType::UBX, TSeries>::getConfigResult()
{
    const UbxConfigResult raw = UbxGNSS::ubxConfigResult();

    // All fields explicitly assigned — zero-initialise first so
    // CASIC-specific fields are demonstrably zeroed.
    GnssConfigResult result = {};
    result.status           = raw.status;           // UbxConfigStatus == GnssConfigStatus
    result.detectedBaud     = raw.detectedBaud;
    result.validationPassed = raw.validationPassed;
    result.detectedProvider = raw.detectedProvider; // UBX-specific
    result.protocolVersion  = raw.protocolVersion;  // UBX-specific
    // observedProtoMask, observedIntervalMs, observedBaudRate remain zero
    return result;
}


/**
* @brief  CASIC specialisation.
*         TProtocol is accepted but unused; the default value allows
*         Gnss<GnssType::CASIC> to be written without a second argument.
*/
template<UbxSeries TSeries>
class Gnss<GnssType::CASIC, TSeries> : private CasicGNSS
{
  public:
    Gnss() = default;

    bool  begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
                { return CasicGNSS::begin(serial, rxPin, txPin);       }
    void  beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                    uint32_t baud = CasicConfigurator::TARGET_BAUD_RATE)
                    {CasicGNSS::beginPassive(serial, rxPin, txPin, baud);}
    void  update()       { CasicGNSS::update();      }
    bool  hasNewData()   { return CasicGNSS::hasNewData();  }
    bool  isFixValid()   { return CasicGNSS::isFixValid();  }
    void  getData(GnssData& dest)   { CasicGNSS::getData(dest);  }
    GnssConfigResult getConfigResult();
};

/**
* @brief  CASIC specialisation.
*         ConfigurationResult translation
*/
template<UbxSeries TSeries>
GnssConfigResult Gnss<GnssType::CASIC, TSeries>::getConfigResult()
{
    const CasicConfigResult raw = CasicGNSS::casicConfigResult();

    GnssConfigResult result = {};
    result.status              = raw.status;
    result.detectedBaud        = raw.detectedBaud;
    result.validationPassed    = raw.validationPassed;
    result.observedProtoMask   = raw.observedProtoMask;   // CASIC-specific
    result.observedIntervalMs  = raw.observedIntervalMs;  // CASIC-specific
    result.observedBaudRate    = raw.observedBaudRate;    // CASIC-specific
    // detectedProvider, protocolVersion remain zero/default
    return result;
}