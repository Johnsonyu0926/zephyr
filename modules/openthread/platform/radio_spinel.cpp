/*
 *  Copyright (c) 2021, The OpenThread Authors.
 *  Copyright (c) 2022-2024, NXP.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * This file implements OpenThread platform driver API in openthread/platform/radio.h.
 *
 */

#include <openthread/platform/radio.h>
#include <lib/platform/exit_code.h>
#include <lib/spinel/radio_spinel.hpp>
#include <lib/spinel/spinel.h>
#include <lib/url/url.hpp>
#include "hdlc_interface.hpp"

static ot::Spinel::RadioSpinel   *psRadioSpinel;
static ot::Url::Url              *psRadioUrl;
static ot::Hdlc::HdlcInterface *pSpinelInterface;
static ot::Spinel::SpinelDriver  *psSpinelDriver;

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(psRadioSpinel->Get(SPINEL_PROP_HWADDR, SPINEL_DATATYPE_EUI64_S, aIeeeEui64));
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(psRadioSpinel->SetPanId(panid));
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	otExtAddress addr;

	for (size_t i = 0; i < sizeof(addr); i++)
	{
		addr.m8[i] = aAddress->m8[sizeof(addr) - 1 - i];
	}

	SuccessOrDie(psRadioSpinel->SetExtendedAddress(addr));
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(psRadioSpinel->SetShortAddress(aAddress));
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(psRadioSpinel->SetPromiscuous(aEnable));
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->IsEnabled();
}

otError otPlatRadioEnable(otInstance *aInstance)
{
	return psRadioSpinel->Enable(aInstance);
}

otError otPlatRadioDisable(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->Disable();
}

otError otPlatRadioSleep(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->Sleep();
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->Receive(aChannel);
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->Transmit(*aFrame);
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return &psRadioSpinel->GetTransmitFrame();
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->GetRssi();
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->GetRadioCaps();
}

const char *otPlatRadioGetVersionString(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->GetVersion();
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->IsPromiscuous();
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(psRadioSpinel->EnableSrcMatch(aEnable));
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->AddSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	otExtAddress addr;

	for (size_t i = 0; i < sizeof(addr); i++)
	{
		addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
	}

	return psRadioSpinel->AddSrcMatchExtEntry(addr);
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->ClearSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	otExtAddress addr;

	for (size_t i = 0; i < sizeof(addr); i++)
	{
		addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
	}

	return psRadioSpinel->ClearSrcMatchExtEntry(addr);
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(psRadioSpinel->ClearSrcMatchShortEntries());
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(psRadioSpinel->ClearSrcMatchExtEntries());
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->EnergyScan(aScanChannel, aScanDuration);
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
	otError error;

	OT_UNUSED_VARIABLE(aInstance);
	VerifyOrExit(aPower != NULL, error = OT_ERROR_INVALID_ARGS);
	error = psRadioSpinel->GetTransmitPower(*aPower);

exit:
	return error;
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->SetTransmitPower(aPower);
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
	otError error;

	OT_UNUSED_VARIABLE(aInstance);
	VerifyOrExit(aThreshold != NULL, error = OT_ERROR_INVALID_ARGS);
	error = psRadioSpinel->GetCcaEnergyDetectThreshold(*aThreshold);

exit:
	return error;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->SetCcaEnergyDetectThreshold(aThreshold);
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->GetReceiveSensitivity();
}

#if OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE
otError otPlatRadioSetCoexEnabled(otInstance *aInstance, bool aEnabled)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->SetCoexEnabled(aEnabled);
}

bool otPlatRadioIsCoexEnabled(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->IsCoexEnabled();
}

otError otPlatRadioGetCoexMetrics(otInstance *aInstance, otRadioCoexMetrics *aCoexMetrics)
{
	OT_UNUSED_VARIABLE(aInstance);

	otError error = OT_ERROR_NONE;

	VerifyOrExit(aCoexMetrics != NULL, error = OT_ERROR_INVALID_ARGS);

	error = psRadioSpinel->GetCoexMetrics(*aCoexMetrics);

exit:
	return error;
}
#endif

#if OPENTHREAD_CONFIG_DIAG_ENABLE
otError otPlatDiagProcess(otInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
	/* Deliver the platform specific diags commands to radio only ncp */
	OT_UNUSED_VARIABLE(aInstance);
	char  cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE] = {'\0'};
	char *cur											   = cmd;
	char *end											   = cmd + sizeof(cmd);

	for (int index = 0; index < argc; index++)
	{
		cur += snprintf(cur, static_cast<size_t>(end - cur), "%s ", argv[index]);
	}

	return psRadioSpinel->PlatDiagProcess(cmd, aOutput, aOutputMaxLen);
}

void otPlatDiagModeSet(bool aMode)
{
	SuccessOrExit(psRadioSpinel->PlatDiagProcess(aMode ? "start" : "stop", NULL, 0));
	psRadioSpinel->SetDiagEnabled(aMode);

exit:
	return;
}

bool otPlatDiagModeGet(void)
{
	return psRadioSpinel->IsDiagEnabled();
}

void otPlatDiagTxPowerSet(int8_t aTxPower)
{
	char cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE];

	snprintf(cmd, sizeof(cmd), "power %d", aTxPower);
	SuccessOrExit(psRadioSpinel->PlatDiagProcess(cmd, NULL, 0));

exit:
	return;
}

void otPlatDiagChannelSet(uint8_t aChannel)
{
	char cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE];

	snprintf(cmd, sizeof(cmd), "channel %d", aChannel);
	SuccessOrExit(psRadioSpinel->PlatDiagProcess(cmd, NULL, 0));

exit:
	return;
}

void otPlatDiagRadioReceived(otInstance *aInstance, otRadioFrame *aFrame, otError aError)
{
	OT_UNUSED_VARIABLE(aInstance);
	OT_UNUSED_VARIABLE(aFrame);
	OT_UNUSED_VARIABLE(aError);
}

void otPlatDiagAlarmCallback(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
}
#endif /* OPENTHREAD_CONFIG_DIAG_ENABLE */

uint32_t otPlatRadioGetSupportedChannelMask(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->GetRadioChannelMask(false);
}

uint32_t otPlatRadioGetPreferredChannelMask(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->GetRadioChannelMask(true);
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->GetState();
}

void otPlatRadioSetMacKey(otInstance			 *aInstance,
						  uint8_t				 aKeyIdMode,
						  uint8_t				 aKeyId,
						  const otMacKeyMaterial *aPrevKey,
						  const otMacKeyMaterial *aCurrKey,
						  const otMacKeyMaterial *aNextKey,
						  otRadioKeyType		  aKeyType)
{
	SuccessOrDie(psRadioSpinel->SetMacKey(aKeyIdMode, aKeyId, aPrevKey, aCurrKey, aNextKey));
	OT_UNUSED_VARIABLE(aInstance);
	OT_UNUSED_VARIABLE(aKeyType);
}

void otPlatRadioSetMacFrameCounter(otInstance *aInstance, uint32_t aMacFrameCounter)
{
	SuccessOrDie(psRadioSpinel->SetMacFrameCounter(aMacFrameCounter, false));
	OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioSetMacFrameCounterIfLarger(otInstance *aInstance, uint32_t aMacFrameCounter)
{
	SuccessOrDie(psRadioSpinel->SetMacFrameCounter(aMacFrameCounter, true));
	OT_UNUSED_VARIABLE(aInstance);
}

uint64_t otPlatRadioGetNow(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->GetNow();
}

uint8_t otPlatRadioGetCslAccuracy(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);

	return psRadioSpinel->GetCslAccuracy();
}

uint8_t otPlatRadioGetCslUncertainty(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);

	return psRadioSpinel->GetCslUncertainty();
}

otError otPlatRadioSetChannelMaxTransmitPower(otInstance *aInstance, uint8_t aChannel, int8_t aMaxPower)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->SetChannelMaxTransmitPower(aChannel, aMaxPower);
}

otError otPlatRadioSetRegion(otInstance *aInstance, uint16_t aRegionCode)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->SetRadioRegion(aRegionCode);
}

otError otPlatRadioGetRegion(otInstance *aInstance, uint16_t *aRegionCode)
{
	OT_UNUSED_VARIABLE(aInstance);
	return psRadioSpinel->GetRadioRegion(aRegionCode);
}

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
otError otPlatRadioConfigureEnhAckProbing(otInstance          *aInstance,
                                          otLinkMetrics        aLinkMetrics,
                                          const otShortAddress aShortAddress,
                                          const otExtAddress  *aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    return psRadioSpinel->ConfigureEnhAckProbing(aLinkMetrics, aShortAddress, *aExtAddress);
}
#endif

otError otPlatRadioReceiveAt(otInstance *aInstance, uint8_t aChannel, uint32_t aStart, uint32_t aDuration)
{
	OT_UNUSED_VARIABLE(aInstance);
	OT_UNUSED_VARIABLE(aChannel);
	OT_UNUSED_VARIABLE(aStart);
	OT_UNUSED_VARIABLE(aDuration);
	return OT_ERROR_NOT_IMPLEMENTED;
}

extern "C" void platformRadioInit(void)
{
	spinel_iid_t iidList[ot::Spinel::kSpinelHeaderMaxNumIid];
	struct ot::Spinel::RadioSpinelCallbacks callbacks;

	iidList[0] = 0;

	psRadioSpinel = new ot::Spinel::RadioSpinel();
	psSpinelDriver = new ot::Spinel::SpinelDriver();

	psRadioUrl = new ot::Url::Url();
	pSpinelInterface = new ot::Hdlc::HdlcInterface(*psRadioUrl);

	OT_UNUSED_VARIABLE(
		psSpinelDriver->Init(*pSpinelInterface, true /* aSoftwareReset */, iidList, OT_ARRAY_LENGTH(iidList)));

	memset(&callbacks, 0, sizeof(callbacks));
#if OPENTHREAD_CONFIG_DIAG_ENABLE
	callbacks.mDiagReceiveDone  = otPlatDiagRadioReceiveDone;
	callbacks.mDiagTransmitDone = otPlatDiagRadioTransmitDone;
#endif /* OPENTHREAD_CONFIG_DIAG_ENABLE */
	callbacks.mEnergyScanDone = otPlatRadioEnergyScanDone;
	callbacks.mReceiveDone	= otPlatRadioReceiveDone;
	callbacks.mTransmitDone   = otPlatRadioTxDone;
	callbacks.mTxStarted	  = otPlatRadioTxStarted;

	psRadioSpinel->SetCallbacks(callbacks);
	psRadioSpinel->Init(false, true, psSpinelDriver);
}

extern "C" void platformRadioDeinit(void)
{
	psRadioSpinel->Deinit();
	psSpinelDriver->Deinit();
}

extern "C" void platformRadioProcess(const otInstance *aInstance)
{
	psSpinelDriver->Process(aInstance);
	psRadioSpinel->Process(aInstance);
}
