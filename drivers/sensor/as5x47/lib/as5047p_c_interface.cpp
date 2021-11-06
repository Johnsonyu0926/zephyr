//
// Created by jonasotto on 11/2/21.
//

#include "as5047p_c_interface.h"

#include <logging/log.h>
#include <AS5047P/src/AS5047P.h>

LOG_MODULE_REGISTER(as5047p_c_interface, LOG_LEVEL_WRN);

/**
 * Log error and set out-parameter
 * @param errorOut[out] Will be set to true if an error has occured. Can be nullptr.
 */
void handleError(const AS5047P_Types::ERROR_t &error, bool *errorOut) {
    if (errorOut != nullptr) {
        *errorOut = !error.noError();
    }

    if (!error.noError()) {
        LOG_ERR("C_GENERAL_COM_ERR: %d, C_SPI_PARITY_ERR: %d, C_WRITE_VERIFY_FAILED: %d, S_CORDIC_OVERFLOW_ERR: %d, "
                "S_OFFSET_COMP_ERR: %d, S_MAG_TOO_HIGH: %d, S_MAG_TOO_LOW: %d, S_SPI_FRAMING_ERR: %d, "
                "S_SPI_INVALID_CMD: %d, S_SPI_PARITY_ERR: %d",
                error.controllerSideErrors.flags.CONT_GENERAL_COM_ERROR,
                error.controllerSideErrors.flags.CONT_SPI_PARITY_ERROR,
                error.controllerSideErrors.flags.CONT_WRITE_VERIFY_FAILED,
                error.sensorSideErrors.flags.SENS_CORDIC_OVERFLOW_ERROR,
                error.sensorSideErrors.flags.SENS_OFFSET_COMP_ERROR,
                error.sensorSideErrors.flags.SENS_MAG_TOO_HIGH,
                error.sensorSideErrors.flags.SENS_MAG_TOO_LOW,
                error.sensorSideErrors.flags.SENS_SPI_FRAMING_ERROR,
                error.sensorSideErrors.flags.SENS_SPI_INVALID_CMD,
                error.sensorSideErrors.flags.SENS_SPI_PARITY_ERROR);
    }
}

extern "C" {

// Init --------------------------------------------------------

bool initSPI(const AS5047P_handle h) {
    AS5047P dev(h);
    return dev.initSPI();
}

// Util --------------------------------------------------------

//std::string readStatusAsStdString(const AS5047P_handle h) {

//}

// -------------------------------------------------------------

// Read High-Level ---------------------------------------------

uint16_t readMagnitude(const AS5047P_handle h, bool *errorOut, bool verifyParity, bool checkForComError,
                       bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    auto mag = dev.readMagnitude(&error, verifyParity, checkForComError, checkForSensorError);

    return mag;
}

uint16_t readAngleRaw(const AS5047P_handle h, bool withDAEC, bool *errorOut, bool verifyParity, bool checkForComError,
                      bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    auto angle = dev.readAngleRaw(withDAEC, &error, verifyParity, checkForComError, checkForSensorError);
    handleError(error, errorOut);

    return angle;
}

float readAngleDegree(const AS5047P_handle h, bool withDAEC, bool *errorOut, bool verifyParity, bool checkForComError,
                      bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    auto angle = dev.readAngleDegree(withDAEC, &error, verifyParity, checkForComError, checkForSensorError);
    handleError(error, errorOut);

    return angle;
}

// -------------------------------------------------------------

// Read Volatile Registers -------------------------------------

bool read_ERRFL(const AS5047P_handle h, as5047p_ERRFL_data_t *regData, bool verifyParity, bool checkForComError,
                bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_ERRFL(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

bool read_PROG(const AS5047P_handle h, as5047p_PROG_data_t *regData, bool verifyParity, bool checkForComError,
               bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_PROG(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

bool read_DIAAGC(const AS5047P_handle h, as5047p_DIAAGC_data_t *regData, bool verifyParity, bool checkForComError,
                 bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_DIAAGC(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

bool read_MAG(const AS5047P_handle h, as5047p_MAG_data_t *regData, bool verifyParity, bool checkForComError,
              bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_MAG(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

bool read_ANGLEUNC(const AS5047P_handle h, as5047p_ANGLEUNC_data_t *regData, bool verifyParity, bool checkForComError,
                   bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_ANGLEUNC(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

bool read_ANGLECOM(const AS5047P_handle h, as5047p_ANGLECOM_data_t *regData, bool verifyParity, bool checkForComError,
                   bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_ANGLECOM(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

// -------------------------------------------------------------

// Write Volatile Registers ------------------------------------

bool
write_PROG(const AS5047P_handle h, const as5047p_PROG_data_t *regData, bool checkForComError, bool verifyWrittenReg) {
    if (regData == nullptr) {
        return false;
    }
    AS5047P_Types::PROG_t reg;
    reg.data.raw = regData->raw;
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    bool res = dev.write_PROG(&reg, &error, checkForComError, verifyWrittenReg);
    handleError(error, nullptr);

    return res;
}

// -------------------------------------------------------------

// Read Non-Volatile Registers ---------------------------------

bool read_ZPOSM(const AS5047P_handle h, as5047p_ZPOSM_data_t *regData, bool verifyParity, bool checkForComError,
                bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_ZPOSM(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

bool read_ZPOSL(const AS5047P_handle h, as5047p_ZPOSL_data_t *regData, bool verifyParity, bool checkForComError,
                bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_ZPOSL(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

bool read_SETTINGS1(const AS5047P_handle h, as5047p_SETTINGS1_data_t *regData, bool verifyParity, bool checkForComError,
                    bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_SETTINGS1(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

bool read_SETTINGS2(const AS5047P_handle h, as5047p_SETTINGS2_data_t *regData, bool verifyParity, bool checkForComError,
                    bool checkForSensorError) {
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    if (regData != nullptr) {
        regData->raw = dev.read_SETTINGS2(&error, verifyParity, checkForComError, checkForSensorError).data.raw;
    }
    handleError(error, nullptr);

    return error.noError();
}

// -------------------------------------------------------------

// Write Non-Volatile Registers --------------------------------

bool
write_ZPOSM(const AS5047P_handle h, const as5047p_ZPOSM_data_t *regData, bool checkForComError, bool verifyWrittenReg) {
    if (regData == nullptr) {
        return false;
    }
    AS5047P_Types::ZPOSM_t reg;
    reg.data.raw = regData->raw;
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    bool res = dev.write_ZPOSM(&reg, &error, checkForComError, verifyWrittenReg);
    handleError(error, nullptr);

    return res;
}

bool
write_ZPOSL(const AS5047P_handle h, const as5047p_ZPOSL_data_t *regData, bool checkForComError, bool verifyWrittenReg) {
    if (regData == nullptr) {
        return false;
    }
    AS5047P_Types::ZPOSL_t reg;
    reg.data.raw = regData->raw;
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    bool res = dev.write_ZPOSL(&reg, &error, checkForComError, verifyWrittenReg);
    handleError(error, nullptr);

    return res;
}

bool write_SETTINGS1(const AS5047P_handle h, const as5047p_SETTINGS1_data_t *regData, bool checkForComError,
                     bool verifyWittenReg) {
    if (regData == nullptr) {
        return false;
    }
    AS5047P_Types::SETTINGS1_t reg;
    reg.data.raw = regData->raw;
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    bool res = dev.write_SETTINGS1(&reg, &error, checkForComError, verifyWittenReg);
    handleError(error, nullptr);

    return res;
}

bool write_SETTINGS2(const AS5047P_handle h, const as5047p_SETTINGS2_data_t *regData, bool checkForComError,
                     bool verifyWrittenReg) {
    if (regData == nullptr) {
        return false;
    }
    AS5047P_Types::SETTINGS2_t reg;
    reg.data.raw = regData->raw;
    AS5047P dev(h);
    AS5047P_Types::ERROR_t error;
    bool res = dev.write_SETTINGS2(&reg, &error, checkForComError, verifyWrittenReg);
    handleError(error, nullptr);

    return res;
}

// -------------------------------------------------------------

}
