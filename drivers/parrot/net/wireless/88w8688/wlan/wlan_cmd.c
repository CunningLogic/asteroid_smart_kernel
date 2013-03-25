/** @file wlan_cmd.c
  *  
  * @brief This file contains the handling of command.
  * it prepares command and sends it to firmware when
  * it is ready.
  * 
  * Copyright (C) 2003-2008, Marvell International Ltd. 
  *
  * This software file (the "File") is distributed by Marvell International 
  * Ltd. under the terms of the GNU General Public License Version 2, June 1991 
  * (the "License").  You may use, redistribute and/or modify this File in 
  * accordance with the terms and conditions of the License, a copy of which 
  * is available along with the File in the gpl.txt file or by writing to 
  * the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 
  * 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
  *
  * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
  * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
  * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
  * this warranty disclaimer.
  *
  */
/********************************************************
Change log:
    10/04/05: Add Doxygen format comments
    01/05/06: Add kernel 2.6.x support  
    01/11/06: Conditionalize new scan/join structures
    01/31/06: Add support to selectively enable the FW Scan channel filter
    02/16/06: Clear scan in progress flag when scan command failed and dropped
    04/06/06: Add TSPEC, queue metrics, and MSDU expiry support
    04/18/06: Remove old Subscribe Event and add new Subscribe Event
              implementation through generic hostcmd API
    05/04/06: Add IBSS coalescing related new hostcmd handling	      
    08/29/06: Add ledgpio private command
********************************************************/

#include	"wlan_headers.h"

/********************************************************
		Local Variables
********************************************************/

static u16 Commands_Allowed_In_PS[] = {
    HostCmd_CMD_RSSI_INFO,
    HostCmd_CMD_802_11_HOST_SLEEP_CFG,
    HostCmd_CMD_802_11_WAKEUP_CONFIRM,
    HostCmd_CMD_MAC_CONTROL,
    HostCmd_CMD_MAC_MULTICAST_ADR,
    HostCmd_CMD_802_11_MAC_ADDRESS,
    HostCmd_CMD_802_11_TX_RATE_QUERY,
};

/********************************************************
		Global Variables
********************************************************/

/********************************************************
		Local Functions
********************************************************/

/** 
 *  @brief This function checks if the commands is allowed
 *  in PS mode not.
 *  
 *  @param Command the command ID
 *  @return 	   TRUE or FALSE
 */
static BOOLEAN
wlan_is_cmd_allowed_in_ps(u16 Command)
{
    int count = sizeof(Commands_Allowed_In_PS)
        / sizeof(Commands_Allowed_In_PS[0]);
    int i;

    for (i = 0; i < count; i++)
        if (Command == wlan_cpu_to_le16(Commands_Allowed_In_PS[i]))
            return TRUE;

    return FALSE;
}

/** 
 *  @brief This function prepares command of get_hw_spec.
 *  
 *  @param priv    A pointer to wlan_private structure
 *  @param cmd	   A pointer to HostCmd_DS_COMMAND structure
 *  @return 	   WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_hw_spec(wlan_private * priv, HostCmd_DS_COMMAND * cmd)
{
    HostCmd_DS_GET_HW_SPEC *hwspec = &cmd->params.hwspec;

    ENTER();

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_GET_HW_SPEC);
    cmd->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_GET_HW_SPEC) + S_DS_GEN);
    memcpy(hwspec->PermanentAddr, priv->adapter->CurrentAddr, ETH_ALEN);

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of ps_mode.
 *  
 *  @param priv    	A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action 	the action: GET or SET
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_ps_mode(wlan_private * priv,
                        HostCmd_DS_COMMAND * cmd, u16 cmd_action)
{
    HostCmd_DS_802_11_PS_MODE *psm = &cmd->params.psmode;
    u16 Action = cmd_action;
    wlan_adapter *Adapter = priv->adapter;

    ENTER();

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_PS_MODE);
    cmd->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_PS_MODE) + S_DS_GEN);
    psm->Action = wlan_cpu_to_le16(cmd_action);
    psm->MultipleDtim = 0;
    switch (Action) {
    case HostCmd_SubCmd_Enter_PS:
    case HostCmd_SubCmd_Enter_PS_Poll:
        PRINTM(INFO, "PS Command:" "SubCode- Enter PS\n");
        PRINTM(INFO, "LocalListenInterval = %d\n",
               Adapter->LocalListenInterval);

        psm->LocalListenInterval =
            wlan_cpu_to_le16(Adapter->LocalListenInterval);
        psm->NullPktInterval = wlan_cpu_to_le16(Adapter->NullPktInterval);
        psm->MultipleDtim = wlan_cpu_to_le16(priv->adapter->MultipleDtim);
        psm->BCNMissTimeOut = wlan_cpu_to_le16(priv->adapter->BCNMissTimeOut);
        if (priv->adapter->InfrastructureMode == Wlan802_11IBSS)
            psm->AdhocAwakePeriod =
                wlan_cpu_to_le16(priv->adapter->AdhocAwakePeriod);
        break;

    case HostCmd_SubCmd_Exit_PS:
        PRINTM(INFO, "PS Command:" "SubCode- Exit PS\n");
        break;

    case HostCmd_SubCmd_Sleep_Confirmed:
        PRINTM(INFO, "PS Command: SubCode- sleep confirm\n");
        break;

    default:
        break;
    }

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of fw_wakeup_method.
 *  
 *  @param priv    	A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action 	the action: GET or SET
 *  @param pdata_buf 	A pointer to data buffer
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_fw_wakeup_method(wlan_private * priv,
                                 HostCmd_DS_COMMAND * cmd,
                                 int cmd_action, void *pdata_buf)
{
    HostCmd_DS_802_11_FW_WAKEUP_METHOD *fwwm = &cmd->params.fwwakeupmethod;
    u16 action = (u16) cmd_action;
    u16 method = *((u16 *) pdata_buf);

    ENTER();
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_FW_WAKE_METHOD);
    cmd->Size =
        wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_FW_WAKEUP_METHOD) + S_DS_GEN);
    fwwm->Action = wlan_cpu_to_le16(action);
    switch (action) {
    case HostCmd_ACT_GEN_SET:
        fwwm->Method = wlan_cpu_to_le16(method);
        break;
    case HostCmd_ACT_GEN_GET:
    default:
        fwwm->Method = 0;
        break;
    }
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function sends the HS_Activated event to the application
 *  
 *  @param priv    	A pointer to wlan_private structure
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int
wlan_host_sleep_activated_event(wlan_private * priv)
{
    ENTER();

    priv->adapter->HS_Activated = TRUE;
    wake_up_interruptible(&priv->adapter->HS_wait_q);
    os_carrier_off(priv);
    os_stop_queue(priv);
    wmm_stop_queue(priv);

#if WIRELESS_EXT > 14
    send_iwevcustom_event(priv, CUS_EVT_HS_ACTIVATED);
#endif /* WIRELESS_EXT */

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function sends the HS_DeActivated event to the application
 *  
 *  @param priv    	A pointer to wlan_private structure
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int
wlan_host_sleep_deactivated_event(wlan_private * priv)
{
    ENTER();

    priv->adapter->HS_Activated = FALSE;

#if WIRELESS_EXT > 14
    send_iwevcustom_event(priv, CUS_EVT_HS_DEACTIVATED);
#endif /* WIRELESS_EXT */

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function sends the HS_GPIO_INT event to the application
 *   
 *  @param priv                 A pointer to wlan_private structure
 *  @return 	   		WLAN_STATUS_SUCCESS --success, otherwise fail 
 */
int
wlan_host_sleep_wakeup_event(wlan_private * priv)
{
    wlan_adapter *Adapter = priv->adapter;
    int ret = WLAN_STATUS_SUCCESS;

    ENTER();

    if (Adapter->bHostSleepConfigured) {
#if WIRELESS_EXT > 14
        send_iwevcustom_event(priv, CUS_EVT_HS_WAKEUP);
#endif /* WIRELESS_EXT */
    } else {
        PRINTM(INFO, "hs_gpio_int: HS not configured !!!\n");
    }

    LEAVE();

    return ret;
}

/** 
 *  @brief This function prepares command of host_sleep_cfg.
 *  
 *  @param priv    	A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf 	A pointer to HostCmd_DS_802_11_HOST_SLEEP_CFG structure
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_host_sleep_cfg(wlan_private * priv,
                               HostCmd_DS_COMMAND * cmd,
                               HostCmd_DS_802_11_HOST_SLEEP_CFG * pdata_buf)
{
    HostCmd_DS_802_11_HOST_SLEEP_CFG *phwuc = &cmd->params.hscfg;

    ENTER();

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_HOST_SLEEP_CFG);
    if ((pdata_buf->conditions != HOST_SLEEP_CFG_CANCEL)
        && ((priv->adapter->ArpFilterSize > 0)
            && (priv->adapter->ArpFilterSize <= ARP_FILTER_MAX_BUF_SIZE))) {
        PRINTM(INFO, "Attach %d bytes ArpFilter to HSCfg cmd\n",
               priv->adapter->ArpFilterSize);
        memcpy(((u8 *) phwuc) + sizeof(HostCmd_DS_802_11_HOST_SLEEP_CFG),
               priv->adapter->ArpFilter, priv->adapter->ArpFilterSize);
        cmd->Size =
            wlan_cpu_to_le16(priv->adapter->ArpFilterSize +
                             sizeof(HostCmd_DS_802_11_HOST_SLEEP_CFG) +
                             S_DS_GEN);
    } else
        cmd->Size =
            wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_HOST_SLEEP_CFG) +
                             S_DS_GEN);
    phwuc->conditions = wlan_cpu_to_le32(pdata_buf->conditions);
    phwuc->gpio = pdata_buf->gpio;
    phwuc->gap = pdata_buf->gap;

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of sleep_period.
 *  
 *  @param priv    		A pointer to wlan_private structure
 *  @param cmd	   		A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action 		the action: GET or SET
 *  @param pdata_buf		A pointer to data buffer
 *  @return 	   		WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_sleep_period(wlan_private * priv,
                             HostCmd_DS_COMMAND * cmd,
                             u16 cmd_action, void *pdata_buf)
{
    HostCmd_DS_802_11_SLEEP_PERIOD *pSleepPeriod = &cmd->params.ps_sleeppd;

    ENTER();
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_SLEEP_PERIOD);
    cmd->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_SLEEP_PERIOD) +
                                 S_DS_GEN);
    memmove(pSleepPeriod, pdata_buf, sizeof(HostCmd_DS_802_11_SLEEP_PERIOD));
    pSleepPeriod->Action = wlan_cpu_to_le16(pSleepPeriod->Action);
    pSleepPeriod->Period = wlan_cpu_to_le16(pSleepPeriod->Period);
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of wlan_cmd_802_11_crypto
 *    
 *  @param priv    		A pointer to wlan_private structure
 *  @param cmd	   		A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action 		the action: GET or SET
 *  @param pdata_buf		A pointer to data buffer
 *  @return 	   		WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_crypto(wlan_private * priv,
                       HostCmd_DS_COMMAND * cmd,
                       u16 cmd_action, void *pdata_buf)
{

    HostCmd_DS_802_11_CRYPTO *crypto = (HostCmd_DS_802_11_CRYPTO *) pdata_buf;
    HostCmd_DS_802_11_CRYPTO_AES_CCM *crypto_aes_ccm;
    MrvlIEtypes_Data_t *data = NULL;
    wlan_adapter *Adapter = priv->adapter;
    int len;

    ENTER();

    if (crypto->Algorithm == CIPHER_TEST_AES_CCM) {
        data =
            (MrvlIEtypes_Data_t *) ((u8 *) pdata_buf +
                                    sizeof(HostCmd_DS_802_11_CRYPTO_AES_CCM));
    } else {
        data =
            (MrvlIEtypes_Data_t *) ((u8 *) pdata_buf +
                                    sizeof(HostCmd_DS_802_11_CRYPTO));
    }
    Adapter->Algorithm = crypto->Algorithm;

    if (data->Header.Type != TLV_TYPE_CRYPTO_DATA)
        return WLAN_STATUS_FAILURE;
    switch (crypto->Algorithm) {
    case CIPHER_TEST_RC4:
        if (((crypto->KeyIVLength + crypto->KeyLength) > 256) ||
            ((crypto->KeyIVLength + crypto->KeyLength) == 0))
            return WLAN_STATUS_FAILURE;
        break;
    case CIPHER_TEST_AES:
        if ((crypto->KeyLength != 16) && (crypto->KeyLength != 24) &&
            (crypto->KeyLength != 32))
            return WLAN_STATUS_FAILURE;
        break;
    case CIPHER_TEST_AES_KEY_WRAP:
        if (crypto->KeyIVLength != 8)
            return WLAN_STATUS_FAILURE;
        if ((crypto->KeyLength != 16) && (crypto->KeyLength != 24) &&
            (crypto->KeyLength != 32))
            return WLAN_STATUS_FAILURE;
        if (data->Header.Len % 8)
            return WLAN_STATUS_FAILURE;
        break;
    case CIPHER_TEST_AES_CCM:
        {
            crypto_aes_ccm = (HostCmd_DS_802_11_CRYPTO_AES_CCM *) pdata_buf;
            if ((crypto_aes_ccm->KeyLength != 16) &&
                (crypto_aes_ccm->KeyLength != 24) &&
                (crypto_aes_ccm->KeyLength != 32))
                return WLAN_STATUS_FAILURE;
            if (!
                (crypto_aes_ccm->NonceLength >= 7 &&
                 crypto_aes_ccm->NonceLength <= 13))
                return WLAN_STATUS_FAILURE;
            if (!
                (crypto_aes_ccm->AADLength >= 1 &&
                 crypto_aes_ccm->AADLength <= 30))
                return WLAN_STATUS_FAILURE;

        }
        break;
    default:
        return WLAN_STATUS_FAILURE;

    }
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_CRYPTO);

    len = data->Header.Len;
    data->Header.Type = wlan_cpu_to_le16(data->Header.Type);
    data->Header.Len = wlan_cpu_to_le16(data->Header.Len);

    if (crypto->Algorithm == CIPHER_TEST_AES_CCM) {
        crypto_aes_ccm = (HostCmd_DS_802_11_CRYPTO_AES_CCM *) pdata_buf;
        crypto_aes_ccm->EncDec = wlan_cpu_to_le16(crypto_aes_ccm->EncDec);
        crypto_aes_ccm->Algorithm = wlan_cpu_to_le16(crypto_aes_ccm->Algorithm);
        crypto_aes_ccm->KeyLength = wlan_cpu_to_le16(crypto_aes_ccm->KeyLength);
        crypto_aes_ccm->NonceLength =
            wlan_cpu_to_le16(crypto_aes_ccm->NonceLength);
        crypto_aes_ccm->AADLength = wlan_cpu_to_le16(crypto_aes_ccm->AADLength);

        memmove(&cmd->params.crypto_aes_ccm, pdata_buf,
                sizeof(HostCmd_DS_802_11_CRYPTO_AES_CCM) + len +
                sizeof(MrvlIEtypesHeader_t));

        cmd->Size =
            wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_CRYPTO_AES_CCM) +
                             S_DS_GEN + len + sizeof(MrvlIEtypesHeader_t));
    } else {
        crypto->EncDec = wlan_cpu_to_le16(crypto->EncDec);
        crypto->Algorithm = wlan_cpu_to_le16(crypto->Algorithm);
        crypto->KeyIVLength = wlan_cpu_to_le16(crypto->KeyIVLength);
        crypto->KeyLength = wlan_cpu_to_le16(crypto->KeyLength);

        memmove(&cmd->params.crypto, pdata_buf,
                sizeof(HostCmd_DS_802_11_CRYPTO) + len +
                sizeof(MrvlIEtypesHeader_t));

        cmd->Size =
            wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_CRYPTO) + S_DS_GEN + len +
                             sizeof(MrvlIEtypesHeader_t));
    }

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of sleep_params.
 *  
 *  @param priv    		A pointer to wlan_private structure
 *  @param cmd	   		A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action 		the action: GET or SET
 *  @return 	   		WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_sleep_params(wlan_private * priv,
                             HostCmd_DS_COMMAND * cmd, u16 cmd_action)
{
    wlan_adapter *Adapter = priv->adapter;
    HostCmd_DS_802_11_SLEEP_PARAMS *sp = &cmd->params.sleep_params;

    ENTER();

    cmd->Size = wlan_cpu_to_le16((sizeof(HostCmd_DS_802_11_SLEEP_PARAMS)) +
                                 S_DS_GEN);
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_SLEEP_PARAMS);

    if (cmd_action == HostCmd_ACT_GEN_GET) {
        memset(&Adapter->sp, 0, sizeof(SleepParams));
        memset(sp, 0, sizeof(HostCmd_DS_802_11_SLEEP_PARAMS));
        sp->Action = wlan_cpu_to_le16(cmd_action);
    } else if (cmd_action == HostCmd_ACT_GEN_SET) {
        sp->Action = wlan_cpu_to_le16(cmd_action);
        sp->Error = wlan_cpu_to_le16(Adapter->sp.sp_error);
        sp->Offset = wlan_cpu_to_le16(Adapter->sp.sp_offset);
        sp->StableTime = wlan_cpu_to_le16(Adapter->sp.sp_stabletime);
        sp->CalControl = (u8) Adapter->sp.sp_calcontrol;
        sp->ExternalSleepClk = (u8) Adapter->sp.sp_extsleepclk;
        sp->Reserved = wlan_cpu_to_le16(Adapter->sp.sp_reserved);
    }

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** Length of WEP 40 bit key */
#define WEP_40_BIT_LEN	5
/** Length of WEP 104 bit key */
#define WEP_104_BIT_LEN	13

/** 
 *  @brief This function prepares command of set_wep.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   HostCmd_ACT_ADD or HostCmd_ACT_REMOVE
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_set_wep(wlan_private * priv,
                        HostCmd_DS_COMMAND * cmd, u16 cmd_action)
{
    HostCmd_DS_802_11_SET_WEP *wep = &cmd->params.wep;
    wlan_adapter *Adapter = priv->adapter;
    int ret = WLAN_STATUS_SUCCESS;

    ENTER();

    if (cmd_action == HostCmd_ACT_ADD) {
        cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_SET_WEP);
        cmd->Size =
            wlan_cpu_to_le16((sizeof(HostCmd_DS_802_11_SET_WEP)) + S_DS_GEN);
        wep->Action = wlan_cpu_to_le16(HostCmd_ACT_ADD);

        /* default tx key index */
        wep->KeyIndex = wlan_cpu_to_le16(Adapter->CurrentWepKeyIndex &
                                         HostCmd_WEP_KEY_INDEX_MASK);

        PRINTM(INFO, "Tx Key Index: %u\n", wep->KeyIndex);

        switch (Adapter->WepKey[0].KeyLength) {
        case WEP_40_BIT_LEN:
            wep->WEPTypeForKey1 = HostCmd_TYPE_WEP_40_BIT;
            memmove(wep->WEP1, Adapter->WepKey[0].KeyMaterial,
                    Adapter->WepKey[0].KeyLength);
            break;
        case WEP_104_BIT_LEN:
            wep->WEPTypeForKey1 = HostCmd_TYPE_WEP_104_BIT;
            memmove(wep->WEP1, Adapter->WepKey[0].KeyMaterial,
                    Adapter->WepKey[0].KeyLength);
            break;
        case 0:
            break;
        default:
            PRINTM(INFO, "Key1 Length = %d is incorrect\n",
                   Adapter->WepKey[0].KeyLength);
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }

        switch (Adapter->WepKey[1].KeyLength) {
        case WEP_40_BIT_LEN:
            wep->WEPTypeForKey2 = HostCmd_TYPE_WEP_40_BIT;
            memmove(wep->WEP2, Adapter->WepKey[1].KeyMaterial,
                    Adapter->WepKey[1].KeyLength);
            break;
        case WEP_104_BIT_LEN:
            wep->WEPTypeForKey2 = HostCmd_TYPE_WEP_104_BIT;
            memmove(wep->WEP2, Adapter->WepKey[1].KeyMaterial,
                    Adapter->WepKey[1].KeyLength);
            break;
        case 0:
            break;
        default:
            PRINTM(INFO, "Key2 Length = %d is incorrect\n",
                   Adapter->WepKey[1].KeyLength);
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }

        switch (Adapter->WepKey[2].KeyLength) {
        case WEP_40_BIT_LEN:
            wep->WEPTypeForKey3 = HostCmd_TYPE_WEP_40_BIT;
            memmove(wep->WEP3, Adapter->WepKey[2].KeyMaterial,
                    Adapter->WepKey[2].KeyLength);
            break;
        case WEP_104_BIT_LEN:
            wep->WEPTypeForKey3 = HostCmd_TYPE_WEP_104_BIT;
            memmove(wep->WEP3, Adapter->WepKey[2].KeyMaterial,
                    Adapter->WepKey[2].KeyLength);
            break;
        case 0:
            break;
        default:
            PRINTM(INFO, "Key3 Length = %d is incorrect\n",
                   Adapter->WepKey[2].KeyLength);
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }

        switch (Adapter->WepKey[3].KeyLength) {
        case WEP_40_BIT_LEN:
            wep->WEPTypeForKey4 = HostCmd_TYPE_WEP_40_BIT;
            memmove(wep->WEP4, Adapter->WepKey[3].KeyMaterial,
                    Adapter->WepKey[3].KeyLength);
            break;
        case WEP_104_BIT_LEN:
            wep->WEPTypeForKey4 = HostCmd_TYPE_WEP_104_BIT;
            memmove(wep->WEP4, Adapter->WepKey[3].KeyMaterial,
                    Adapter->WepKey[3].KeyLength);
            break;
        case 0:
            break;
        default:
            PRINTM(INFO, "Key4 Length = %d is incorrect\n",
                   Adapter->WepKey[3].KeyLength);
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }
    } else if (cmd_action == HostCmd_ACT_REMOVE) {
        cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_SET_WEP);
        cmd->Size =
            wlan_cpu_to_le16((sizeof(HostCmd_DS_802_11_SET_WEP)) + S_DS_GEN);
        wep->Action = wlan_cpu_to_le16(HostCmd_ACT_REMOVE);

        /* default tx key index */
        wep->KeyIndex = wlan_cpu_to_le16((u16) (Adapter->CurrentWepKeyIndex &
                                                (u32)
                                                HostCmd_WEP_KEY_INDEX_MASK));
    }

    ret = WLAN_STATUS_SUCCESS;
  done:
    LEAVE();
    return ret;
}

/** 
 *  @brief This function prepares command of key_material.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action 	the action: GET or SET
 *  @param cmd_oid	OID: ENABLE or DISABLE
 *  @param pdata_buf    A pointer to data buffer
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_key_material(wlan_private * priv,
                             HostCmd_DS_COMMAND * cmd,
                             u16 cmd_action, WLAN_OID cmd_oid, void *pdata_buf)
{
    HostCmd_DS_802_11_KEY_MATERIAL *pKeyMaterial = &cmd->params.keymaterial;
    WLAN_802_11_KEY *pKey = (WLAN_802_11_KEY *) pdata_buf;
    u16 KeyParamSet_len;
    int ret = WLAN_STATUS_SUCCESS;

    ENTER();

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_KEY_MATERIAL);
    pKeyMaterial->Action = wlan_cpu_to_le16(cmd_action);

    if (cmd_action == HostCmd_ACT_GEN_GET) {
        cmd->Size = wlan_cpu_to_le16(2 + S_DS_GEN);
        ret = WLAN_STATUS_SUCCESS;
        goto done;
    }

    memset(&pKeyMaterial->KeyParamSet, 0, sizeof(MrvlIEtype_KeyParamSet_t));
    if (pKey->KeyLength == WPA_AES_KEY_LEN) {
        PRINTM(INFO, "WPA_AES\n");
        pKeyMaterial->KeyParamSet.KeyTypeId = wlan_cpu_to_le16(KEY_TYPE_ID_AES);

        if (cmd_oid == (WLAN_OID) KEY_INFO_ENABLED)
            pKeyMaterial->KeyParamSet.KeyInfo =
                wlan_cpu_to_le16(KEY_INFO_AES_ENABLED);
        else
            pKeyMaterial->KeyParamSet.KeyInfo =
                !(wlan_cpu_to_le16(KEY_INFO_AES_ENABLED));

        if (pKey->KeyIndex & 0x40000000)        /* AES pairwise key: unicast */
            pKeyMaterial->KeyParamSet.KeyInfo |=
                wlan_cpu_to_le16(KEY_INFO_AES_UNICAST);
        else                    /* AES group key: multicast */
            pKeyMaterial->KeyParamSet.KeyInfo |=
                wlan_cpu_to_le16(KEY_INFO_AES_MCAST);
    } else if (pKey->KeyLength == WPA_TKIP_KEY_LEN) {
        PRINTM(INFO, "WPA_TKIP\n");
        pKeyMaterial->KeyParamSet.KeyTypeId =
            wlan_cpu_to_le16(KEY_TYPE_ID_TKIP);
        pKeyMaterial->KeyParamSet.KeyInfo =
            wlan_cpu_to_le16(KEY_INFO_TKIP_ENABLED);

        if (pKey->KeyIndex & 0x40000000)        /* TKIP pairwise key: unicast */
            pKeyMaterial->KeyParamSet.KeyInfo |=
                wlan_cpu_to_le16(KEY_INFO_TKIP_UNICAST);
        else                    /* TKIP group key: multicast */
            pKeyMaterial->KeyParamSet.KeyInfo |=
                wlan_cpu_to_le16(KEY_INFO_TKIP_MCAST);
    }

    if (pKeyMaterial->KeyParamSet.KeyTypeId) {
        pKeyMaterial->KeyParamSet.Type =
            wlan_cpu_to_le16(TLV_TYPE_KEY_MATERIAL);
        pKeyMaterial->KeyParamSet.KeyLen = wlan_cpu_to_le16(pKey->KeyLength);
        memcpy(pKeyMaterial->KeyParamSet.Key,
               pKey->KeyMaterial, pKey->KeyLength);
        pKeyMaterial->KeyParamSet.Length =
            wlan_cpu_to_le16(pKey->KeyLength + 6);

#define TYPE_LEN_FIELDS_LEN 4
        KeyParamSet_len = (pKey->KeyLength + 6) + TYPE_LEN_FIELDS_LEN;
#define ACTION_FIELD_LEN 2
        cmd->Size =
            wlan_cpu_to_le16(KeyParamSet_len + ACTION_FIELD_LEN + S_DS_GEN);
    }

    ret = WLAN_STATUS_SUCCESS;
  done:
    LEAVE();
    return ret;
}

/** 
 *  @brief This function prepares command of get_log.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_get_log(wlan_private * priv, HostCmd_DS_COMMAND * cmd)
{
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_GET_LOG);
    cmd->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_GET_LOG) + S_DS_GEN);

    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of snmp_mib.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @param cmd_oid   	the OID of SNMP MIB
 *  @param pdata_buf	the pointer to data buffer
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_snmp_mib(wlan_private * priv,
                         HostCmd_DS_COMMAND * cmd,
                         int cmd_action, int cmd_oid, void *pdata_buf)
{
    HostCmd_DS_802_11_SNMP_MIB *pSNMPMIB = &cmd->params.smib;
    wlan_adapter *Adapter = priv->adapter;

    ENTER();

    PRINTM(INFO, "SNMP_CMD: cmd_oid = 0x%x\n", cmd_oid);

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_SNMP_MIB);
    cmd->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_SNMP_MIB) + S_DS_GEN);

    switch (cmd_oid) {
    case Dot11D_i:
        {
            u32 ulTemp;

            pSNMPMIB->OID = wlan_cpu_to_le16((u16) Dot11D_i);

            if (cmd_action == HostCmd_ACT_GEN_SET) {
                pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
                pSNMPMIB->BufSize = wlan_cpu_to_le16(sizeof(u16));
                ulTemp = *(u32 *) pdata_buf;
                *((PUSHORT) (pSNMPMIB->Value)) = wlan_cpu_to_le16((u16) ulTemp);
            }
            break;
        }

    case Dot11H_i:
        {
            u32 ulTemp;

            pSNMPMIB->OID = wlan_cpu_to_le16((u16) Dot11H_i);

            if (cmd_action == HostCmd_ACT_GEN_SET) {
                pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
                pSNMPMIB->BufSize = wlan_cpu_to_le16(sizeof(u16));
                ulTemp = *(u32 *) pdata_buf;
                *((PUSHORT) (pSNMPMIB->Value)) = wlan_cpu_to_le16((u16) ulTemp);
            }
            break;
        }
    case DisableAutoChannelSwitch_i:
        {
            u32 ulTemp;

            pSNMPMIB->OID = wlan_cpu_to_le16((u16) DisableAutoChannelSwitch_i);

            if (cmd_action == HostCmd_ACT_GEN_SET) {
                pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
                pSNMPMIB->BufSize = wlan_cpu_to_le16(sizeof(u16));
                ulTemp = *(u32 *) pdata_buf;
                *((PUSHORT) (pSNMPMIB->Value)) = wlan_cpu_to_le16((u16) ulTemp);
            } else if (cmd_action == HostCmd_ACT_GEN_GET) {
                pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_GET);
            }
            break;
        }

    case FragThresh_i:
        {
            WLAN_802_11_FRAGMENTATION_THRESHOLD ulTemp;

            pSNMPMIB->OID = wlan_cpu_to_le16((u16) FragThresh_i);

            if (cmd_action == HostCmd_ACT_GEN_GET) {
                pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_GET);
            } else if (cmd_action == HostCmd_ACT_GEN_SET) {
                pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
                pSNMPMIB->BufSize = wlan_cpu_to_le16(sizeof(u16));
                ulTemp = *((WLAN_802_11_FRAGMENTATION_THRESHOLD *)
                           pdata_buf);
                *((PUSHORT) (pSNMPMIB->Value)) = wlan_cpu_to_le16((u16) ulTemp);

            }

            break;
        }

    case RtsThresh_i:
        {

            WLAN_802_11_RTS_THRESHOLD ulTemp;
            pSNMPMIB->OID = wlan_cpu_to_le16((u16) RtsThresh_i);

            if (cmd_action == HostCmd_ACT_GEN_GET) {
                pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_GET);
            } else if (cmd_action == HostCmd_ACT_GEN_SET) {
                pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
                pSNMPMIB->BufSize = wlan_cpu_to_le16(sizeof(u16));
                ulTemp = *((WLAN_802_11_RTS_THRESHOLD *)
                           pdata_buf);
                *(PUSHORT) (pSNMPMIB->Value) = wlan_cpu_to_le16((u16) ulTemp);

            }
            break;
        }
    case ShortRetryLim_i:
        pSNMPMIB->OID = wlan_cpu_to_le16((u16) ShortRetryLim_i);

        if (cmd_action == HostCmd_ACT_GEN_GET) {
            pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_GET);
        } else if (cmd_action == HostCmd_ACT_GEN_SET) {
            pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
            pSNMPMIB->BufSize = wlan_cpu_to_le16(sizeof(u16));
            *((PUSHORT) (pSNMPMIB->Value)) =
                wlan_cpu_to_le16((u16) Adapter->TxRetryCount);
        }
        break;

    case DtimPeriod_i:
        pSNMPMIB->OID = wlan_cpu_to_le16((u16) DtimPeriod_i);

        if (cmd_action == HostCmd_ACT_GEN_GET) {
            pSNMPMIB->QueryType = wlan_cpu_to_le16(HostCmd_ACT_GEN_GET);
        }
        break;

    default:
        break;
    }

    PRINTM(INFO, "SNMP_CMD: Action=0x%x, OID=0x%x, OIDSize=0x%x, Value=0x%x\n",
           cmd_action, cmd_oid, wlan_le16_to_cpu(pSNMPMIB->BufSize),
           wlan_le16_to_cpu(*(u16 *) pSNMPMIB->Value));

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of radio_control.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_radio_control(wlan_private * priv,
                              HostCmd_DS_COMMAND * cmd,
                              int cmd_action, void *pdata_buf)
{
    HostCmd_DS_802_11_RADIO_CONTROL *pRadioControl = &cmd->params.radio;

    ENTER();

    cmd->Size = wlan_cpu_to_le16((sizeof(HostCmd_DS_802_11_RADIO_CONTROL))
                                 + S_DS_GEN);
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_RADIO_CONTROL);

    pRadioControl->Action = wlan_cpu_to_le16(cmd_action);
    pRadioControl->Control = wlan_cpu_to_le16(*(u16 *) pdata_buf);

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of bca_timeshare.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @param user_bca_ts	A pointer to HostCmd_DS_802_11_BCA_TIMESHARE structure
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_bca_timeshare(wlan_private * priv,
                              HostCmd_DS_COMMAND * cmd,
                              u16 cmd_action,
                              HostCmd_DS_802_11_BCA_TIMESHARE * user_bca_ts)
{
    wlan_adapter *Adapter = priv->adapter;
    HostCmd_DS_802_11_BCA_TIMESHARE *bca_ts = &cmd->params.bca_timeshare;

    ENTER();

    cmd->Size = wlan_cpu_to_le16((sizeof(HostCmd_DS_802_11_BCA_TIMESHARE)) +
                                 S_DS_GEN);
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_BCA_CONFIG_TIMESHARE);

    if (cmd_action == HostCmd_ACT_GEN_GET) {
        memset(&Adapter->bca_ts, 0, sizeof(bca_ts));
        memset(bca_ts, 0, sizeof(HostCmd_DS_802_11_BCA_TIMESHARE));
        bca_ts->Action = wlan_cpu_to_le16(cmd_action);
        bca_ts->TrafficType = wlan_cpu_to_le16(user_bca_ts->TrafficType);
    } else if (cmd_action == HostCmd_ACT_GEN_SET) {
        bca_ts->Action = wlan_cpu_to_le16(cmd_action);
        bca_ts->TrafficType = wlan_cpu_to_le16(user_bca_ts->TrafficType);
        bca_ts->TimeShareInterval =
            wlan_cpu_to_le32(user_bca_ts->TimeShareInterval);
        bca_ts->BTTime = wlan_cpu_to_le32(user_bca_ts->BTTime);
    }

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of rf_tx_power.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @param pdata_buf	A pointer to data buffer
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_rf_tx_power(wlan_private * priv,
                            HostCmd_DS_COMMAND * cmd,
                            u16 cmd_action, void *pdata_buf)
{

    HostCmd_DS_802_11_RF_TX_POWER *pRTP = &cmd->params.txp;

    ENTER();

    cmd->Size =
        wlan_cpu_to_le16((sizeof(HostCmd_DS_802_11_RF_TX_POWER)) + S_DS_GEN);
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_RF_TX_POWER);
    pRTP->Action = cmd_action;

    PRINTM(INFO, "RF_TX_POWER_CMD: Size:%d Cmd:0x%x Act:%d\n", cmd->Size,
           cmd->Command, pRTP->Action);

    switch (cmd_action) {
    case HostCmd_ACT_GEN_GET:
        pRTP->Action = wlan_cpu_to_le16(HostCmd_ACT_GEN_GET);
        pRTP->CurrentLevel = 0;
        break;

    case HostCmd_ACT_GEN_SET:
        pRTP->Action = wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
        pRTP->CurrentLevel = wlan_cpu_to_le16(*((u16 *) pdata_buf));
        break;
    }
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of rf_antenna.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @param pdata_buf	A pointer to data buffer
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_rf_antenna(wlan_private * priv,
                           HostCmd_DS_COMMAND * cmd,
                           u16 cmd_action, void *pdata_buf)
{
    HostCmd_DS_802_11_RF_ANTENNA *rant = &cmd->params.rant;

    ENTER();
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_RF_ANTENNA);
    cmd->Size =
        wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_RF_ANTENNA) + S_DS_GEN);

    rant->Action = wlan_cpu_to_le16(cmd_action);
    if ((cmd_action == HostCmd_ACT_SET_RX) ||
        (cmd_action == HostCmd_ACT_SET_TX)) {
        rant->AntennaMode =
            wlan_cpu_to_le16((u16) (*(WLAN_802_11_ANTENNA *) pdata_buf));
    }
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of rate_adapt_rateset.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_rate_adapt_rateset(wlan_private * priv,
                                   HostCmd_DS_COMMAND * cmd, u16 cmd_action)
{
    HostCmd_DS_802_11_RATE_ADAPT_RATESET * rateadapt = &cmd->params.rateset;
    wlan_adapter *Adapter = priv->adapter;

    cmd->Size =
        wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_RATE_ADAPT_RATESET) +
                         S_DS_GEN);
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_RATE_ADAPT_RATESET);

    ENTER();

    rateadapt->Action = wlan_cpu_to_le16(cmd_action);
    rateadapt->HWRateDropMode = wlan_cpu_to_le16(Adapter->HWRateDropMode);
    rateadapt->Threshold = wlan_cpu_to_le16(Adapter->Threshold);
    rateadapt->FinalRate = wlan_cpu_to_le16(Adapter->FinalRate);
    rateadapt->Bitmap = wlan_cpu_to_le16(Adapter->RateBitmap);

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of mac_multicast_adr.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_mac_multicast_adr(wlan_private * priv,
                           HostCmd_DS_COMMAND * cmd, u16 cmd_action)
{
    HostCmd_DS_MAC_MULTICAST_ADR *pMCastAdr = &cmd->params.madr;
    wlan_adapter *Adapter = priv->adapter;

    ENTER();
    cmd->Size =
        wlan_cpu_to_le16(sizeof(HostCmd_DS_MAC_MULTICAST_ADR) + S_DS_GEN);
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_MAC_MULTICAST_ADR);

    pMCastAdr->Action = wlan_cpu_to_le16(cmd_action);
    pMCastAdr->NumOfAdrs =
        wlan_cpu_to_le16((u16) Adapter->NumOfMulticastMACAddr);
    memcpy(pMCastAdr->MACList, Adapter->MulticastList,
           Adapter->NumOfMulticastMACAddr * ETH_ALEN);
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of rf_channel.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param option	the action: GET or SET
 *  @param pdata_buf	A pointer to data buffer
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_rf_channel(wlan_private * priv,
                           HostCmd_DS_COMMAND * cmd,
                           int option, void *pdata_buf)
{
    HostCmd_DS_802_11_RF_CHANNEL *rfchan = &cmd->params.rfchannel;

    ENTER();
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_RF_CHANNEL);
    cmd->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_RF_CHANNEL)
                                 + S_DS_GEN);

    if (option == HostCmd_OPT_802_11_RF_CHANNEL_SET) {
        if (priv->adapter->adhoc_start_band == BAND_A)
            rfchan->RFType = wlan_cpu_to_le16(HostCmd_SCAN_RADIO_TYPE_A);
        rfchan->CurrentChannel = wlan_cpu_to_le16(*((u16 *) pdata_buf));
    }

    rfchan->Action = wlan_cpu_to_le16(option);
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of rssi info.
 *  
 *  @param priv       	A pointer to wlan_private structure
 *  @param cmd        	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action 	Command action
 *  @param wait_option	Wait response or not
 *  @return           	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_rssi_info(wlan_private * priv,
                          HostCmd_DS_COMMAND * cmd,
                          u16 cmd_action, u16 wait_option)
{
    wlan_adapter *Adapter = priv->adapter;

    ENTER();

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_RSSI_INFO);
    cmd->Size =
        wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_RSSI_INFO) + S_DS_GEN);
    cmd->params.rssi_info.Action = wlan_cpu_to_le16(cmd_action);
    cmd->params.rssi_info.Ndata = wlan_cpu_to_le16(Adapter->data_avg_factor);
    cmd->params.rssi_info.Nbcn = wlan_cpu_to_le16(Adapter->bcn_avg_factor);

    if (wait_option & HostCmd_OPTION_WAITFORRSP) {
        /* reset Beacon SNR/NF/RSSI values */
        Adapter->DataRSSIlast = 0;
        Adapter->DataNFlast = 0;
        Adapter->DataRSSIAvg = 0;
        Adapter->DataNFAvg = 0;
        Adapter->BcnRSSIlast = 0;
        Adapter->BcnNFlast = 0;
        Adapter->BcnRSSIAvg = 0;
        Adapter->BcnNFAvg = 0;
    }

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of reg_access.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param CmdPtr  	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @param pdata_buf	A pointer to data buffer
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_reg_access(wlan_private * priv,
                    HostCmd_DS_COMMAND * CmdPtr, u8 cmd_action, void *pdata_buf)
{
    wlan_offset_value *offval;

    ENTER();

    offval = (wlan_offset_value *) pdata_buf;

    switch (CmdPtr->Command) {
    case HostCmd_CMD_MAC_REG_ACCESS:
        {
            HostCmd_DS_MAC_REG_ACCESS *macreg;

            CmdPtr->Size =
                wlan_cpu_to_le16(sizeof(HostCmd_DS_MAC_REG_ACCESS) + S_DS_GEN);
            macreg = (HostCmd_DS_MAC_REG_ACCESS *) & CmdPtr->params.macreg;

            macreg->Action = wlan_cpu_to_le16(cmd_action);
            macreg->Offset = wlan_cpu_to_le16((u16) offval->offset);
            macreg->Value = wlan_cpu_to_le32(offval->value);

            break;
        }

    case HostCmd_CMD_BBP_REG_ACCESS:
        {
            HostCmd_DS_BBP_REG_ACCESS *bbpreg;

            CmdPtr->Size =
                wlan_cpu_to_le16(sizeof(HostCmd_DS_BBP_REG_ACCESS) + S_DS_GEN);
            bbpreg = (HostCmd_DS_BBP_REG_ACCESS *) & CmdPtr->params.bbpreg;

            bbpreg->Action = wlan_cpu_to_le16(cmd_action);
            bbpreg->Offset = wlan_cpu_to_le16((u16) offval->offset);
            bbpreg->Value = (u8) offval->value;

            break;
        }

    case HostCmd_CMD_RF_REG_ACCESS:
        {
            HostCmd_DS_RF_REG_ACCESS *rfreg;

            CmdPtr->Size =
                wlan_cpu_to_le16(sizeof(HostCmd_DS_RF_REG_ACCESS) + S_DS_GEN);
            rfreg = (HostCmd_DS_RF_REG_ACCESS *) & CmdPtr->params.rfreg;

            rfreg->Action = wlan_cpu_to_le16(cmd_action);
            rfreg->Offset = wlan_cpu_to_le16((u16) offval->offset);
            rfreg->Value = (u8) offval->value;

            break;
        }
    case HostCmd_CMD_PMIC_REG_ACCESS:
        {
            HostCmd_DS_PMIC_REG_ACCESS *pmicReg;

            CmdPtr->Size =
                wlan_cpu_to_le16(sizeof(HostCmd_DS_PMIC_REG_ACCESS) + S_DS_GEN);
            pmicReg = (HostCmd_DS_PMIC_REG_ACCESS *) & CmdPtr->params.pmicreg;

            pmicReg->Action = wlan_cpu_to_le16(cmd_action);
            pmicReg->Offset = wlan_cpu_to_le16((u16) offval->offset);
            pmicReg->Value = (u8) offval->value;

            break;
        }
    default:
        break;
    }

    CmdPtr->Command = wlan_cpu_to_le16(CmdPtr->Command);

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of mac_address.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_mac_address(wlan_private * priv,
                            HostCmd_DS_COMMAND * cmd, u16 cmd_action)
{
    wlan_adapter *Adapter = priv->adapter;
    ENTER();
    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_MAC_ADDRESS);
    cmd->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_MAC_ADDRESS) +
                                 S_DS_GEN);
    cmd->Result = 0;

    cmd->params.macadd.Action = wlan_cpu_to_le16(cmd_action);

    if (cmd_action == HostCmd_ACT_GEN_SET) {
        memcpy(cmd->params.macadd.MacAdd, Adapter->CurrentAddr, ETH_ALEN);
        HEXDUMP("SET_CMD: MAC ADDRESS-", Adapter->CurrentAddr, 6);
    }
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of eeprom_access.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @param pdata_buf	A pointer to data buffer
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_802_11_eeprom_access(wlan_private * priv,
                              HostCmd_DS_COMMAND * cmd,
                              int cmd_action, void *pdata_buf)
{
    wlan_ioctl_regrdwr *ea = pdata_buf;

    ENTER();

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_EEPROM_ACCESS);
    cmd->Size =
        wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_EEPROM_ACCESS) + S_DS_GEN);
    cmd->Result = 0;

    cmd->params.rdeeprom.Action = wlan_cpu_to_le16(ea->Action);
    cmd->params.rdeeprom.Offset = wlan_cpu_to_le16(ea->Offset);
    cmd->params.rdeeprom.ByteCount = wlan_cpu_to_le16(ea->NOB);
    cmd->params.rdeeprom.Value = 0;

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

static int
wlan_cmd_ibss_coalescing_status(wlan_private * priv,
                                HostCmd_DS_COMMAND * cmd,
                                int cmd_action, void *pdata_buf)
{
    HostCmd_DS_802_11_IBSS_Status *pIBSSReq = &(cmd->params.ibssCoalescing);
    u16 *enable = pdata_buf;

    ENTER();
    PRINTM(INFO, "HostCmd_CMD_802_11_BSSID_QUERY request");

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_IBSS_COALESCING_STATUS);
    cmd->Size =
        wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_IBSS_Status) + S_DS_GEN);
    cmd->Result = 0;
    pIBSSReq->Action = wlan_cpu_to_le16(cmd_action);

    switch (cmd_action) {
    case HostCmd_ACT_GEN_SET:
        pIBSSReq->Enable = wlan_cpu_to_le16(*enable);
        break;

        /* In other case.. Noting to do */
    case HostCmd_ACT_GEN_GET:
    default:
        break;
    }
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function prepares command of mgmt IE list.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd	   	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   the action: GET or SET
 *  @param pdata_buf 	A pointer to data buffer
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 *
 */
static int
wlan_cmd_mgmt_ie_list(wlan_private * priv,
                      HostCmd_DS_COMMAND * cmd, int cmd_action, void *pdata_buf)
{
    u16 req_len = 0, travel_len = 0;
    custom_ie *cptr = NULL;
    wlan_ioctl_custom_ie *cust_ie = NULL;
    HostCmd_DS_MGMT_IE_LIST_CFG *pmgmt_ie_list = &(cmd->params.mgmt_ie_list);

    ENTER();

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_MGMT_IE_LIST);
    cmd->Size = sizeof(HostCmd_DS_MGMT_IE_LIST_CFG) + S_DS_GEN;
    cmd->Result = 0;
    pmgmt_ie_list->action = wlan_cpu_to_le16(cmd_action);

    cust_ie = (wlan_ioctl_custom_ie *) pdata_buf;
    pmgmt_ie_list->ds_mgmt_ie.type = wlan_cpu_to_le16(cust_ie->type);
    pmgmt_ie_list->ds_mgmt_ie.len = wlan_cpu_to_le16(cust_ie->len);

    if (pmgmt_ie_list->ds_mgmt_ie.ie_data_list && cust_ie->ie_data_list) {
        req_len = cust_ie->len;
        travel_len = 0;
        /* conversion for index, mask, len */
        if (req_len == sizeof(u16))
            cust_ie->ie_data_list[0].ie_index =
                wlan_cpu_to_le16(cust_ie->ie_data_list[0].ie_index);

        while (req_len > sizeof(u16)) {
            cptr = (custom_ie *) (((u8 *) cust_ie->ie_data_list) + travel_len);
            travel_len += cptr->ie_length + sizeof(custom_ie) - MAX_IE_SIZE;
            req_len -= cptr->ie_length + sizeof(custom_ie) - MAX_IE_SIZE;
            cptr->ie_index = wlan_cpu_to_le16(cptr->ie_index);
            cptr->mgmt_subtype_mask = wlan_cpu_to_le16(cptr->mgmt_subtype_mask);
            cptr->ie_length = wlan_cpu_to_le16(cptr->ie_length);
        }
        memcpy(pmgmt_ie_list->ds_mgmt_ie.ie_data_list, cust_ie->ie_data_list,
               cust_ie->len);
    }

    cmd->Size -= MAX_MGMT_IE_INDEX * sizeof(custom_ie);
    cmd->Size += cust_ie->len;
    cmd->Size = wlan_cpu_to_le16(cmd->Size);

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function queues the command to cmd list.
 *  
 *  @param Adapter	A pointer to wlan_adapter structure
 *  @param CmdNode   	A pointer to CmdCtrlNode structure
 *  @param addtail	specify if the cmd needs to be queued in the header or tail
 *  @return 	   	n/a
 */
void
wlan_insert_cmd_to_pending_q(wlan_adapter * Adapter, CmdCtrlNode * CmdNode,
                             BOOLEAN addtail)
{
    ulong flags;
    HostCmd_DS_COMMAND *CmdPtr;
    u16 command;

    ENTER();

    if (!CmdNode) {
        PRINTM(WARN, "QUEUE_CMD: CmdNode is NULL\n");
        goto done;
    }

    CmdPtr = (HostCmd_DS_COMMAND *) CmdNode->BufVirtualAddr;
    if (!CmdPtr) {
        PRINTM(WARN, "QUEUE_CMD: CmdPtr is NULL\n");
        goto done;
    }

    command = wlan_le16_to_cpu(CmdPtr->Command);

    /* Exit_PS command needs to be queued in the header always. */
    if (command == HostCmd_CMD_802_11_PS_MODE) {
        HostCmd_DS_802_11_PS_MODE *psm = &CmdPtr->params.psmode;
        if (wlan_le16_to_cpu(psm->Action) == HostCmd_SubCmd_Exit_PS) {
            if (Adapter->PSState != PS_STATE_FULL_POWER)
                addtail = FALSE;
        }
    }

    if ((command == HostCmd_CMD_802_11_WAKEUP_CONFIRM)
        || (command == HostCmd_CMD_802_11_HOST_SLEEP_ACTIVATE)
        || (command == HostCmd_CMD_802_11_HOST_SLEEP_CFG)
        ) {
        addtail = FALSE;
    }

    spin_lock_irqsave(&Adapter->QueueSpinLock, flags);

    if (addtail)
        list_add_tail((struct list_head *) CmdNode, &Adapter->CmdPendingQ);
    else
        list_add((struct list_head *) CmdNode, &Adapter->CmdPendingQ);

    spin_unlock_irqrestore(&Adapter->QueueSpinLock, flags);

    PRINTM(INFO, "QUEUE_CMD: cmd=0x%x is queued\n", command);

  done:
    LEAVE();
    return;
}

#ifdef MFG_CMD_SUPPORT
/** 
 *  @brief This function sends general command to firmware.
 *  
 *  @param priv     	A pointer to wlan_private structure
 *  @param cmd      	A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf	A pointer to data buffer
 *  @return         	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_mfg_cmd(wlan_private * priv, HostCmd_DS_COMMAND * cmd, void *pdata_buf)
{
    HostCmd_DS_GEN *pCmdPtr;

    ENTER();

    pCmdPtr = (HostCmd_DS_GEN *) pdata_buf;

    /* copy the MFG command to command buffer */
    memcpy((void *) cmd, pdata_buf, pCmdPtr->Size);

    PRINTM(INFO, "MFG command size = %d\n", pCmdPtr->Size);

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_MFG_COMMAND);
    cmd->Size = wlan_cpu_to_le16(cmd->Size);
    cmd->Result = 0;

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}
#endif

/** 
 *  @brief This function downloads the command to firmware.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param CmdNode   	A pointer to CmdCtrlNode structure
 *  @return 	   	WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_dnld_cmd_to_fw(wlan_private * priv, CmdCtrlNode * CmdNode)
{
    ulong flags;
    HostCmd_DS_COMMAND *CmdPtr;
    wlan_adapter *Adapter = priv->adapter;
    int ret = WLAN_STATUS_SUCCESS;
    u16 CmdSize;
    u16 Command;
    unsigned long driver_flags;

    OS_INTERRUPT_SAVE_AREA;

    ENTER();

    if (!Adapter || !CmdNode) {
        PRINTM(ERROR, "DNLD_CMD: Adapter = %#x, CmdNode = %#x\n", (int) Adapter,
               (int) CmdNode);
        if (CmdNode)
            wlan_insert_cmd_to_free_q(priv, CmdNode);
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    CmdPtr = (HostCmd_DS_COMMAND *) CmdNode->BufVirtualAddr;

    if (!CmdPtr || !CmdPtr->Size) {
        PRINTM(ERROR, "DNLD_CMD: CmdPtr is Null or Cmd Size is Zero, "
               "Not sending\n");
        wlan_insert_cmd_to_free_q(priv, CmdNode);
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    /* Set command sequence number */
    Adapter->SeqNum++;
    CmdPtr->SeqNum = wlan_cpu_to_le16(Adapter->SeqNum);

    spin_lock_irqsave(&Adapter->QueueSpinLock, flags);
    Adapter->CurCmd = CmdNode;
    spin_unlock_irqrestore(&Adapter->QueueSpinLock, flags);

    Command = wlan_le16_to_cpu(CmdPtr->Command);
    CmdSize = wlan_le16_to_cpu(CmdPtr->Size);

    CmdNode->CmdWaitQWoken = FALSE;

    ret = sbi_host_to_card(priv, MV_TYPE_CMD, (u8 *) CmdPtr, CmdSize);

    /* clear TxDone interrupt bit */
    OS_INT_DISABLE(priv, driver_flags);
    Adapter->HisRegCpy &= ~HIS_TxDnLdRdy;
    OS_INT_RESTORE(priv, driver_flags);

    if (ret != 0) {
        PRINTM(ERROR, "DNLD_CMD: Host to Card Failed\n");
        /* set error code that will be transferred back to wlan_prepare_cmd() */
        Adapter->CurCmdRetCode = WLAN_STATUS_FAILURE;
        wlan_insert_cmd_to_free_q(priv, Adapter->CurCmd);
        spin_lock_irqsave(&Adapter->QueueSpinLock, flags);
        Adapter->CurCmd = NULL;
        spin_unlock_irqrestore(&Adapter->QueueSpinLock, flags);
        Adapter->dbg.num_cmd_host_to_card_failure++;
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }
    /* soft_reset command has no command response, we should return here */
    if (Command == HostCmd_CMD_SOFT_RESET) {
        PRINTM(CMND, "DNLD_CMD: SOFT_RESET done\n");
        Adapter->CurCmdRetCode = WLAN_STATUS_SUCCESS;
        wlan_insert_cmd_to_free_q(priv, Adapter->CurCmd);
        spin_lock_irqsave(&Adapter->QueueSpinLock, flags);
        Adapter->CurCmd = NULL;
        spin_unlock_irqrestore(&Adapter->QueueSpinLock, flags);
        ret = WLAN_STATUS_SUCCESS;
        goto done;
    }
    /* Save the last command id and action to debug log */
    Adapter->dbg.LastCmdIndex = (Adapter->dbg.LastCmdIndex + 1) % DBG_CMD_NUM;
    Adapter->dbg.LastCmdId[Adapter->dbg.LastCmdIndex] = Command;
    Adapter->dbg.LastCmdAct[Adapter->dbg.LastCmdIndex] =
        wlan_le16_to_cpu(*(u16 *) ((u8 *) CmdPtr + S_DS_GEN));

    PRINTM(CMND, "DNLD_CMD: 0x%x, act 0x%x, len %d, seqno %d @ %lu\n",
           Command, wlan_le16_to_cpu(*(u16 *) ((u8 *) CmdPtr + S_DS_GEN)),
           CmdSize, wlan_le16_to_cpu(CmdPtr->SeqNum), os_time_get());
    DBG_HEXDUMP(CMD_D, "DNLD_CMD", CmdNode->BufVirtualAddr, CmdSize);

    /* Setup the timer after transmit command */
    if (Command == HostCmd_CMD_802_11_SCAN
        || Command == HostCmd_CMD_802_11_DEAUTHENTICATE
        || Command == HostCmd_CMD_802_11_ASSOCIATE
        || Command == HostCmd_CMD_WMM_ADDTS_REQ)
        wlan_mod_timer(&Adapter->MrvDrvCommandTimer, MRVDRV_TIMER_10S);
    else
        wlan_mod_timer(&Adapter->MrvDrvCommandTimer, MRVDRV_TIMER_5S);

    Adapter->CommandTimerIsSet = TRUE;

    if (Command == HostCmd_CMD_802_11_DEEP_SLEEP) {
        if (Adapter->IntCounter && !Adapter->IsAutoDeepSleepEnabled) {
            PRINTM(INFO, "DNLD_CMD: DS- IntCnt=%d\n", Adapter->IntCounter);
            OS_INT_DISABLE(priv, driver_flags);
            Adapter->IntCounterSaved = Adapter->IntCounter;
            Adapter->IntCounter = 0;
            OS_INT_RESTORE(priv, driver_flags);
        }
        /* 
         * 1. change the PS state to DEEP_SLEEP
         * 2. since there is no response for this command, so 
         *    delete the command timer and free the Node. 
         */

        Adapter->IsDeepSleep = TRUE;

        wlan_insert_cmd_to_free_q(priv, CmdNode);
        spin_lock_irqsave(&Adapter->QueueSpinLock, flags);
        Adapter->CurCmd = NULL;
        spin_unlock_irqrestore(&Adapter->QueueSpinLock, flags);

        if (Adapter->CommandTimerIsSet) {
            wlan_cancel_timer(&Adapter->MrvDrvCommandTimer);
            Adapter->CommandTimerIsSet = FALSE;
        }

        if (Adapter->IsAutoDeepSleepEnabled) {
            Adapter->bWakeupDevRequired = TRUE;
            /* 
             * For auto deep sleep mode, after entering deep sleep state, 
             * cmd_sent flag should be cleared so that the commands in 
             * pending queue can be handled by main thread. 
             */
            priv->wlan_dev.cmd_sent = FALSE;
        }

        if (Adapter->bHostSleepConfigured) {
            Adapter->bWakeupDevRequired = TRUE;
            wlan_host_sleep_activated_event(priv);
        }

    }

    ret = WLAN_STATUS_SUCCESS;

  done:
    LEAVE();
    return ret;
}

/** 
 *  @brief This function prepares command of mac_control.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd		A pointer to HostCmd_DS_COMMAND structure
 *  @param InfoBuf	A pointer to the command string
 *  @return 		WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_cmd_mac_control(wlan_private * priv,
                     HostCmd_DS_COMMAND * cmd, void *InfoBuf)
{
    HostCmd_DS_MAC_CONTROL *mac = &cmd->params.macctrl;
    u16 Action = *((u16 *) InfoBuf);

    ENTER();

    cmd->Command = wlan_cpu_to_le16(HostCmd_CMD_MAC_CONTROL);
    cmd->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_MAC_CONTROL) + S_DS_GEN);
    mac->Action = wlan_cpu_to_le16(Action);

    PRINTM(INFO, "wlan_cmd_mac_control(): Action=0x%X Size=%d\n",
           mac->Action, cmd->Size);

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/********************************************************
		Global Functions
********************************************************/

/** 
 *  @brief This function inserts command node to CmdFreeQ
 *  after cleans it.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param pTempCmd	A pointer to CmdCtrlNode structure
 *  @return 		n/a
 */
void
wlan_insert_cmd_to_free_q(wlan_private * priv, CmdCtrlNode * pTempCmd)
{
    ulong flags;
    wlan_adapter *Adapter = priv->adapter;

    ENTER();

    if (!pTempCmd)
        goto done;

    spin_lock_irqsave(&Adapter->QueueSpinLock, flags);
    wlan_clean_cmd_node(pTempCmd);
    list_add_tail((struct list_head *) pTempCmd, &Adapter->CmdFreeQ);
    spin_unlock_irqrestore(&Adapter->QueueSpinLock, flags);

  done:
    LEAVE();
}

/** 
 *  @brief This function prepare the command before send to firmware.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param cmd_no	command number
 *  @param cmd_action	command action: GET or SET
 *  @param wait_option	wait option: wait response or not
 *  @param cmd_oid	cmd oid: treated as sub command
 *  @param pdata_buf	A pointer to information buffer
 *  @return 		WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int
wlan_prepare_cmd(wlan_private * priv,
                 u16 cmd_no,
                 u16 cmd_action,
                 u16 wait_option, WLAN_OID cmd_oid, void *pdata_buf)
{
    int ret = WLAN_STATUS_SUCCESS;
    wlan_adapter *Adapter = priv->adapter;
    CmdCtrlNode *CmdNode;
    HostCmd_DS_COMMAND *CmdPtr;

    ENTER();

    if (!Adapter) {
        PRINTM(ERROR, "PREP_CMD: Adapter is Null\n");
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    if (Adapter->SurpriseRemoved) {
        PRINTM(ERROR, "PREP_CMD: Card is Removed\n");
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    if (Adapter->HardwareStatus == WlanHardwareStatusReset) {
        if (cmd_no != HostCmd_CMD_FUNC_INIT) {
            PRINTM(ERROR, "Firmware is not ready\n");
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }
    }

    CmdNode = wlan_get_cmd_node(priv);

    if (CmdNode == NULL) {
        PRINTM(MSG, "PREP_CMD: No free CmdNode\n");

        /* Wake up main thread to execute next command */
        wake_up_interruptible(&priv->MainThread.waitQ);
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    wlan_init_cmd_node(priv, CmdNode, cmd_oid, wait_option, pdata_buf);

    CmdPtr = (HostCmd_DS_COMMAND *) CmdNode->BufVirtualAddr;

    if (!CmdPtr) {
        PRINTM(MSG, "PREP_CMD: BufVirtualAddr of CmdNode is NULL\n");
        wlan_insert_cmd_to_free_q(priv, CmdNode);
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    CmdPtr->Command = cmd_no;
    CmdPtr->Result = 0;

    TX_EVENT_FLAGS_SET(&CmdNode->cmdwait_q, 0, TX_AND);
    switch (cmd_no) {
    case HostCmd_CMD_GET_HW_SPEC:
        ret = wlan_cmd_hw_spec(priv, CmdPtr);
        break;
    case HostCmd_CMD_802_11_PS_MODE:
        ret = wlan_cmd_802_11_ps_mode(priv, CmdPtr, cmd_action);
        break;

    case HostCmd_CMD_802_11_SCAN:
        ret = wlan_cmd_802_11_scan(priv, CmdPtr, pdata_buf);
        break;

    case HostCmd_CMD_MAC_CONTROL:
        ret = wlan_cmd_mac_control(priv, CmdPtr, pdata_buf);
        break;

    case HostCmd_CMD_802_11_ASSOCIATE:
        ret = wlan_cmd_802_11_associate(priv, CmdPtr, pdata_buf);
        break;

    case HostCmd_CMD_802_11_DEAUTHENTICATE:
        ret = wlan_cmd_802_11_deauthenticate(priv, CmdPtr, pdata_buf);
        break;

    case HostCmd_CMD_802_11_SET_WEP:
        ret = wlan_cmd_802_11_set_wep(priv, CmdPtr, cmd_action);
        break;

    case HostCmd_CMD_802_11_AD_HOC_START:
        ret = wlan_cmd_802_11_ad_hoc_start(priv, CmdPtr, pdata_buf);
        break;

    case HostCmd_CMD_802_11_GET_LOG:
        ret = wlan_cmd_802_11_get_log(priv, CmdPtr);
        break;

    case HostCmd_CMD_802_11_SNMP_MIB:
        ret = wlan_cmd_802_11_snmp_mib(priv, CmdPtr,
                                       cmd_action, cmd_oid, pdata_buf);
        break;

    case HostCmd_CMD_MAC_REG_ACCESS:
    case HostCmd_CMD_BBP_REG_ACCESS:
    case HostCmd_CMD_RF_REG_ACCESS:
    case HostCmd_CMD_PMIC_REG_ACCESS:
        ret = wlan_cmd_reg_access(priv, CmdPtr, cmd_action, pdata_buf);
        break;

    case HostCmd_CMD_802_11_RF_CHANNEL:
        ret = wlan_cmd_802_11_rf_channel(priv, CmdPtr, cmd_action, pdata_buf);
        break;

    case HostCmd_CMD_802_11_RF_TX_POWER:
        ret = wlan_cmd_802_11_rf_tx_power(priv, CmdPtr, cmd_action, pdata_buf);
        break;

    case HostCmd_CMD_802_11_RADIO_CONTROL:
        ret = wlan_cmd_802_11_radio_control(priv, CmdPtr,
                                            cmd_action, pdata_buf);
        break;

    case HostCmd_CMD_802_11_RF_ANTENNA:
        ret = wlan_cmd_802_11_rf_antenna(priv, CmdPtr, cmd_action, pdata_buf);
        break;

    case HostCmd_CMD_802_11_RATE_ADAPT_RATESET:
        ret = wlan_cmd_802_11_rate_adapt_rateset(priv, CmdPtr, cmd_action);
        break;

    case HostCmd_CMD_MAC_MULTICAST_ADR:
        ret = wlan_cmd_mac_multicast_adr(priv, CmdPtr, cmd_action);
        break;

    case HostCmd_CMD_802_11_AD_HOC_JOIN:
        ret = wlan_cmd_802_11_ad_hoc_join(priv, CmdPtr, pdata_buf);
        break;
    case HostCmd_CMD_RSSI_INFO:
        ret = wlan_cmd_802_11_rssi_info(priv, CmdPtr, cmd_action, wait_option);
        break;
    case HostCmd_CMD_802_11_AD_HOC_STOP:
        ret = wlan_cmd_802_11_ad_hoc_stop(priv, CmdPtr);
        break;
    case HostCmd_CMD_802_11_KEY_MATERIAL:
        ret = wlan_cmd_802_11_key_material(priv, CmdPtr,
                                           cmd_action, cmd_oid, pdata_buf);
        break;

    case HostCmd_CMD_802_11_MAC_ADDRESS:
        ret = wlan_cmd_802_11_mac_address(priv, CmdPtr, cmd_action);
        break;

    case HostCmd_CMD_802_11_DEEP_SLEEP:
        CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
        CmdPtr->Size = wlan_cpu_to_le16((u16)
                                        (sizeof(HostCmd_DS_802_11_DEEP_SLEEP)));
        break;

    case HostCmd_CMD_802_11_HOST_SLEEP_CFG:
        ret = wlan_cmd_802_11_host_sleep_cfg(priv, CmdPtr, pdata_buf);
        break;
    case HostCmd_CMD_802_11_WAKEUP_CONFIRM:
    case HostCmd_CMD_802_11_HOST_SLEEP_ACTIVATE:
        CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
        CmdPtr->Size = wlan_cpu_to_le16(S_DS_GEN);
        break;
    case HostCmd_CMD_802_11_EEPROM_ACCESS:
        ret = wlan_cmd_802_11_eeprom_access(priv, CmdPtr,
                                            cmd_action, pdata_buf);
        break;

#ifdef MFG_CMD_SUPPORT
    case HostCmd_CMD_MFG_COMMAND:
        ret = wlan_cmd_mfg_cmd(priv, CmdPtr, pdata_buf);
        break;
#endif

    case HostCmd_CMD_802_11D_DOMAIN_INFO:
        ret = wlan_cmd_802_11d_domain_info(priv, CmdPtr, cmd_no, cmd_action);
        break;

    case HostCmd_CMD_802_11_TPC_ADAPT_REQ:
    case HostCmd_CMD_802_11_TPC_INFO:
    case HostCmd_CMD_802_11_CHAN_SW_ANN:
        ret = wlan_11h_cmd_process(priv, CmdPtr, pdata_buf);
        break;

    case HostCmd_CMD_MEASUREMENT_REQUEST:
    case HostCmd_CMD_MEASUREMENT_REPORT:
        ret = wlan_meas_cmd_process(priv, CmdPtr, pdata_buf);
        break;

    case HostCmd_CMD_802_11_SLEEP_PARAMS:
        ret = wlan_cmd_802_11_sleep_params(priv, CmdPtr, cmd_action);
        break;
    case HostCmd_CMD_802_11_BCA_CONFIG_TIMESHARE:
        ret = wlan_cmd_802_11_bca_timeshare(priv, CmdPtr,
                                            cmd_action, pdata_buf);
        break;
    case HostCmd_CMD_802_11_BG_SCAN_CONFIG:
        ret = wlan_cmd_802_11_bg_scan_config(priv, CmdPtr,
                                             cmd_action, pdata_buf);
        break;

    case HostCmd_CMD_802_11_BG_SCAN_QUERY:
        ret = wlan_cmd_802_11_bg_scan_query(priv, CmdPtr);
        break;

    case HostCmd_CMD_802_11_FW_WAKE_METHOD:
        ret = wlan_cmd_802_11_fw_wakeup_method(priv, CmdPtr,
                                               cmd_action, pdata_buf);
        break;

    case HostCmd_CMD_MGMT_IE_LIST:
        ret = wlan_cmd_mgmt_ie_list(priv, CmdPtr, cmd_action, pdata_buf);
        break;

    case HostCmd_CMD_WMM_GET_STATUS:
        ret = wlan_cmd_wmm_get_status(priv, CmdPtr, pdata_buf);
        break;
    case HostCmd_CMD_WMM_ADDTS_REQ:
        ret = wlan_cmd_wmm_addts_req(priv, CmdPtr, pdata_buf);
        break;
    case HostCmd_CMD_WMM_DELTS_REQ:
        ret = wlan_cmd_wmm_delts_req(priv, CmdPtr, pdata_buf);
        break;
    case HostCmd_CMD_WMM_QUEUE_CONFIG:
        ret = wlan_cmd_wmm_queue_config(priv, CmdPtr, pdata_buf);
        break;
    case HostCmd_CMD_WMM_QUEUE_STATS:
        ret = wlan_cmd_wmm_queue_stats(priv, CmdPtr, pdata_buf);
        break;
    case HostCmd_CMD_WMM_TS_STATUS:
        ret = wlan_cmd_wmm_ts_status(priv, CmdPtr, pdata_buf);
        break;
    case HostCmd_CMD_TX_PKT_STATS:
        CmdPtr->Command = wlan_cpu_to_le16(HostCmd_CMD_TX_PKT_STATS);
        CmdPtr->Size = wlan_cpu_to_le16(S_DS_GEN);
        ret = WLAN_STATUS_SUCCESS;
        break;
    case HostCmd_CMD_802_11_TPC_CFG:
        CmdPtr->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_TPC_CFG);
        CmdPtr->Size =
            wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_TPC_CFG) + S_DS_GEN);

        memmove(&CmdPtr->params.tpccfg,
                pdata_buf, sizeof(HostCmd_DS_802_11_TPC_CFG));
        CmdPtr->params.tpccfg.Action =
            wlan_cpu_to_le16(CmdPtr->params.tpccfg.Action);

        ret = WLAN_STATUS_SUCCESS;
        break;
    case HostCmd_CMD_802_11_LED_CONTROL:
        {
            HostCmd_DS_802_11_LED_CTRL *pLedCtrl = &CmdPtr->params.ledgpio;
            MrvlIEtypes_LedGpio_t *gpio = &pLedCtrl->LedGpio;
            MrvlIEtypes_LedBehavior_t *pLedBehavior = pLedCtrl->LedBehavior;

            memmove(&CmdPtr->params.ledgpio,
                    pdata_buf, sizeof(HostCmd_DS_802_11_LED_CTRL));

            CmdPtr->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_LED_CONTROL);

#define ACTION_NUMLED_TLVTYPE_LEN_FIELDS_LEN 8
            CmdPtr->Size = wlan_cpu_to_le16(gpio->Header.Len + S_DS_GEN
                                            +
                                            ACTION_NUMLED_TLVTYPE_LEN_FIELDS_LEN);

            pLedCtrl->Action = wlan_cpu_to_le16(pLedCtrl->Action);
            pLedCtrl->LedNums = wlan_cpu_to_le16(pLedCtrl->LedNums);

            gpio->Header.Type = wlan_cpu_to_le16(gpio->Header.Type);
            gpio->Header.Len = wlan_cpu_to_le16(gpio->Header.Len);

            pLedBehavior->Header.Type =
                wlan_cpu_to_le16(pLedBehavior->Header.Type);
            pLedBehavior->Header.Len =
                wlan_cpu_to_le16(pLedBehavior->Header.Len);

            ret = WLAN_STATUS_SUCCESS;
            break;
        }

    case HostCmd_CMD_802_11_SLEEP_PERIOD:
        ret = wlan_cmd_802_11_sleep_period(priv, CmdPtr, cmd_action, pdata_buf);
        break;
    case HostCmd_CMD_802_11_CRYPTO:
        ret = wlan_cmd_802_11_crypto(priv, CmdPtr, cmd_action, pdata_buf);
        break;
    case HostCmd_CMD_GET_TSF:
        CmdPtr->Command = wlan_cpu_to_le16(HostCmd_CMD_GET_TSF);
        CmdPtr->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_GET_TSF)
                                        + S_DS_GEN);
        ret = WLAN_STATUS_SUCCESS;
        break;
    case HostCmd_CMD_802_11_TX_RATE_QUERY:
        CmdPtr->Command = wlan_cpu_to_le16(HostCmd_CMD_802_11_TX_RATE_QUERY);
        CmdPtr->Size =
            wlan_cpu_to_le16(sizeof(HostCmd_TX_RATE_QUERY) + S_DS_GEN);
        Adapter->TxRate = 0;
        ret = WLAN_STATUS_SUCCESS;
        break;
    case HostCmd_CMD_802_11_IBSS_COALESCING_STATUS:
        ret =
            wlan_cmd_ibss_coalescing_status(priv, CmdPtr, cmd_action,
                                            pdata_buf);
        break;

    case HostCmd_CMD_SDIO_PULL_CTRL:
        {
            HostCmd_DS_SDIO_PULL_CTRL *pSdiopullctl =
                &CmdPtr->params.sdiopullctl;

            CmdPtr->Command = wlan_cpu_to_le16(HostCmd_CMD_SDIO_PULL_CTRL);
            CmdPtr->Size =
                wlan_cpu_to_le16(sizeof(HostCmd_DS_SDIO_PULL_CTRL) + S_DS_GEN);

            memcpy(pSdiopullctl, pdata_buf, sizeof(HostCmd_DS_SDIO_PULL_CTRL));
            pSdiopullctl->Action = wlan_cpu_to_le16(pSdiopullctl->Action);
            pSdiopullctl->PullUp = wlan_cpu_to_le16(pSdiopullctl->PullUp);
            pSdiopullctl->PullDown = wlan_cpu_to_le16(pSdiopullctl->PullDown);

            ret = WLAN_STATUS_SUCCESS;
            break;
        }

    case HostCmd_CMD_ECL_SYSTEM_CLOCK_CONFIG:
        {
            HostCmd_DS_ECL_SYSTEM_CLOCK_CONFIG *pSysclockcfg =
                &CmdPtr->params.sysclockcfg;

            CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
            CmdPtr->Size =
                wlan_cpu_to_le16(sizeof(HostCmd_DS_ECL_SYSTEM_CLOCK_CONFIG) +
                                 S_DS_GEN);

            memcpy(pSysclockcfg, pdata_buf,
                   sizeof(HostCmd_DS_ECL_SYSTEM_CLOCK_CONFIG));
            pSysclockcfg->Action = wlan_cpu_to_le16(pSysclockcfg->Action);
            pSysclockcfg->SystemClock =
                wlan_cpu_to_le16(pSysclockcfg->SystemClock);
            break;
        }

    case HostCmd_CMD_802_11_LDO_CONFIG:
        CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
        CmdPtr->Size =
            wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_LDO_CONFIG) + S_DS_GEN);
        memcpy(&CmdPtr->params.ldocfg, pdata_buf,
               sizeof(HostCmd_DS_802_11_LDO_CONFIG));
        CmdPtr->params.ldocfg.Action =
            wlan_cpu_to_le16(CmdPtr->params.ldocfg.Action);
        CmdPtr->params.ldocfg.PMSource =
            wlan_cpu_to_le16(CmdPtr->params.ldocfg.PMSource);
        break;

    case HostCmd_CMD_MODULE_TYPE_CONFIG:
        CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
        CmdPtr->Size =
            wlan_cpu_to_le16(sizeof(HostCmd_DS_MODULE_TYPE_CONFIG) + S_DS_GEN);
        memcpy(&CmdPtr->params.moduletypecfg, pdata_buf,
               sizeof(HostCmd_DS_MODULE_TYPE_CONFIG));
        CmdPtr->params.moduletypecfg.Action =
            wlan_cpu_to_le16(CmdPtr->params.moduletypecfg.Action);
        CmdPtr->params.moduletypecfg.Module =
            wlan_cpu_to_le16(CmdPtr->params.moduletypecfg.Module);
        break;

    case HostCmd_CMD_VERSION_EXT:
        CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
        memcpy(&CmdPtr->params, pdata_buf, sizeof(HostCmd_DS_VERSION_EXT));
        CmdPtr->Size = wlan_cpu_to_le16(sizeof(HostCmd_DS_VERSION_EXT)
                                        + S_DS_GEN);
        break;
    case HostCmd_CMD_MEF_CFG:
        {
            MEF_CFG_DATA *pMefData = (MEF_CFG_DATA *) pdata_buf;
            CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
            memcpy(&CmdPtr->params, (u8 *) & pMefData->data, pMefData->size);
            CmdPtr->Size = wlan_cpu_to_le16(pMefData->size + S_DS_GEN);
            CmdPtr->params.mefcfg.Criteria =
                wlan_cpu_to_le32(CmdPtr->params.mefcfg.Criteria);
            CmdPtr->params.mefcfg.NumEntries =
                wlan_cpu_to_le16(CmdPtr->params.mefcfg.NumEntries);
        }
        break;
    case HostCmd_CMD_INACTIVITY_TIMEOUT_EXT:
        {
            CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
            memcpy(&CmdPtr->params, (u8 *) pdata_buf,
                   sizeof(HostCmd_DS_INACTIVITY_TIMEOUT_EXT));
            CmdPtr->Size =
                wlan_cpu_to_le16(sizeof(HostCmd_DS_INACTIVITY_TIMEOUT_EXT) +
                                 S_DS_GEN);
            CmdPtr->params.inactivityext.Action =
                wlan_cpu_to_le16(CmdPtr->params.inactivityext.Action);
            CmdPtr->params.inactivityext.TimeoutUnit =
                wlan_cpu_to_le16(CmdPtr->params.inactivityext.TimeoutUnit);
            CmdPtr->params.inactivityext.UnicastTimeout =
                wlan_cpu_to_le16(CmdPtr->params.inactivityext.UnicastTimeout);
            CmdPtr->params.inactivityext.MulticastTimeout =
                wlan_cpu_to_le16(CmdPtr->params.inactivityext.MulticastTimeout);
        }
        break;
    case HostCmd_CMD_DBGS_CFG:
        {
            DBGS_CFG_DATA *pDbgCfg = (DBGS_CFG_DATA *) pdata_buf;
            CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
            memcpy(&CmdPtr->params, (u8 *) & pDbgCfg->data, pDbgCfg->size);
            CmdPtr->Size = wlan_cpu_to_le16(pDbgCfg->size + S_DS_GEN);
        }
        break;
    case HostCmd_CMD_GET_MEM:
        {
            FW_MEM_DATA *pFwData = (FW_MEM_DATA *) pdata_buf;
            CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
            memcpy(&CmdPtr->params, (u8 *) & pFwData->data,
                   sizeof(HostCmd_DS_GET_MEM));
            CmdPtr->Size =
                wlan_cpu_to_le16(sizeof(HostCmd_DS_GET_MEM) + S_DS_GEN);
        }
        break;

    case HostCmd_CMD_FUNC_INIT:
        if (Adapter->HardwareStatus == WlanHardwareStatusReset)
            Adapter->HardwareStatus = WlanHardwareStatusReady;
        CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
        CmdPtr->Size = wlan_cpu_to_le16(S_DS_GEN);
        break;
    case HostCmd_CMD_FUNC_SHUTDOWN:
        Adapter->HardwareStatus = WlanHardwareStatusReset;
        CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
        CmdPtr->Size = wlan_cpu_to_le16(S_DS_GEN);
        break;

    case HostCmd_CMD_FUNC_IF_CTRL:
        {
            HostCmd_DS_FUNC_IF_CTRL *intfctrl =
                (HostCmd_DS_FUNC_IF_CTRL *) pdata_buf;
            CmdPtr->Command = wlan_cpu_to_le16(cmd_no);
            CmdPtr->params.intfctrl.Action = wlan_cpu_to_le16(intfctrl->Action);
            CmdPtr->params.intfctrl.Interface =
                wlan_cpu_to_le16(intfctrl->Interface);
            CmdPtr->Size =
                wlan_cpu_to_le16(sizeof(HostCmd_DS_FUNC_IF_CTRL) + S_DS_GEN);
        }
        break;

    case HostCmd_CMD_SOFT_RESET:
        CmdPtr->Command = wlan_cpu_to_le16(HostCmd_CMD_SOFT_RESET);
        CmdPtr->Size = wlan_cpu_to_le16(S_DS_GEN);
        ret = WLAN_STATUS_SUCCESS;
        break;
    case HostCmd_CMD_CTS_POWER_CONTROL:
        CmdPtr->Command = wlan_cpu_to_le16(HostCmd_CMD_CTS_POWER_CONTROL);
        CmdPtr->Size =
            wlan_cpu_to_le16(S_DS_GEN + sizeof(HostCmd_DS_CTS_POWER_CONTROL));
        CmdPtr->params.cts_power_ctrl.Action = wlan_cpu_to_le16(cmd_action);
        CmdPtr->params.cts_power_ctrl.PowerCtrl =
            wlan_cpu_to_le16(*(u16 *) pdata_buf);
        ret = WLAN_STATUS_SUCCESS;
        break;

    default:
        PRINTM(INFO, "PREP_CMD: unknown command- %#x\n", cmd_no);
        ret = WLAN_STATUS_FAILURE;
        break;
    }

    /* return error, since the command preparation failed */
    if (ret != WLAN_STATUS_SUCCESS) {
        PRINTM(ERROR, "PREP_CMD: Command 0x%x preparation failed\n", cmd_no);
        wlan_insert_cmd_to_free_q(priv, CmdNode);
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    PRINTM(CMND, "PREP_CMD: 0x%x\n", cmd_no);

    CmdNode->CmdWaitQWoken = FALSE;
    wlan_insert_cmd_to_pending_q(Adapter, CmdNode, TRUE);
    wake_up_interruptible(&priv->MainThread.waitQ);

    if (wait_option & HostCmd_OPTION_WAITFORRSP) {
        PRINTM(INFO, "PREP_CMD: Wait for CMD response...\n");
        if (wait_option & HostCmd_OPTION_TIMEOUT) {
            if (!os_wait_interruptible_timeout
                (CmdNode->cmdwait_q, CmdNode->CmdWaitQWoken,
                 MRVDRV_TIMER_10S)) {
                PRINTM(INFO, "PREP_CMD:Timeout ...\n");
                ret = WLAN_STATUS_FAILURE;
                goto done;
            }
        } else {
            if (-ERESTARTSYS == wait_event_interruptible(CmdNode->cmdwait_q,
                                                         CmdNode->
                                                         CmdWaitQWoken)) {
                PRINTM(ERROR, "CMD Canceled\n");
                CmdNode->pdata_buf = NULL;
                ret = WLAN_STATUS_FAILURE;
                goto done;
            }
        }
        if (Adapter->CurCmdRetCode) {
            PRINTM(INFO, "PREP_CMD: Command failed with return code=%d\n",
                   Adapter->CurCmdRetCode);
            Adapter->CurCmdRetCode = 0;
            ret = WLAN_STATUS_FAILURE;
        }
    }

  done:
    LEAVE();
    return ret;
}

/** 
 *  @brief This function allocates the command buffer and link
 *  it to command free queue.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @return 		WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int
wlan_alloc_cmd_buffer(wlan_private * priv)
{
    int ret = WLAN_STATUS_SUCCESS;
    u32 ulBufSize;
    u32 i;
    CmdCtrlNode *TempCmdArray;
    u8 *pTempVirtualAddr;
    wlan_adapter *Adapter = priv->adapter;

    ENTER();

    /* Allocate and initialize CmdCtrlNode */
    ulBufSize = sizeof(CmdCtrlNode) * MRVDRV_NUM_OF_CMD_BUFFER;

    if (!(TempCmdArray = kmalloc(ulBufSize, GFP_KERNEL))) {
        PRINTM(INFO, "ALLOC_CMD_BUF: Failed to allocate TempCmdArray\n");
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    Adapter->CmdArray = TempCmdArray;
    memset(Adapter->CmdArray, 0, ulBufSize);

    /* Allocate and initialize command buffers */
    ulBufSize = MRVDRV_SIZE_OF_CMD_BUFFER;
    for (i = 0; i < MRVDRV_NUM_OF_CMD_BUFFER; i++) {
        if (!(pTempVirtualAddr = kmalloc(ulBufSize, GFP_KERNEL))) {
            PRINTM(INFO, "ALLOC_CMD_BUF: pTempVirtualAddr: out of memory\n");
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }

        memset(pTempVirtualAddr, 0, ulBufSize);

        /* Update command buffer virtual */
        TempCmdArray[i].BufVirtualAddr = pTempVirtualAddr;
    }

    for (i = 0; i < MRVDRV_NUM_OF_CMD_BUFFER; i++) {
        init_waitqueue_head(&TempCmdArray[i].cmdwait_q);
        wlan_insert_cmd_to_free_q(priv, &TempCmdArray[i]);
    }

    ret = WLAN_STATUS_SUCCESS;
  done:
    LEAVE();
    return ret;
}

/** 
 *  @brief This function frees the command buffer.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @return 		WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int
wlan_free_cmd_buffer(wlan_private * priv)
{
    u32 ulBufSize;
    UINT i;
    CmdCtrlNode *TempCmdArray;
    wlan_adapter *Adapter = priv->adapter;

    ENTER();

    /* need to check if cmd array is allocated or not */
    if (Adapter->CmdArray == NULL) {
        PRINTM(INFO, "FREE_CMD_BUF: CmdArray is Null\n");
        goto done;
    }

    TempCmdArray = Adapter->CmdArray;

    /* Release shared memory buffers */
    ulBufSize = MRVDRV_SIZE_OF_CMD_BUFFER;
    for (i = 0; i < MRVDRV_NUM_OF_CMD_BUFFER; i++) {
        if (TempCmdArray[i].BufVirtualAddr) {
            PRINTM(INFO, "Free all the array\n");
            kfree(TempCmdArray[i].BufVirtualAddr);
            TempCmdArray[i].BufVirtualAddr = NULL;
        }
    }

    /* Release CmdCtrlNode */
    if (Adapter->CmdArray) {
        PRINTM(INFO, "Free CmdArray\n");
        kfree(Adapter->CmdArray);
        Adapter->CmdArray = NULL;
    }

  done:
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function gets a free command node if available in
 *  command free queue.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @return CmdCtrlNode A pointer to CmdCtrlNode structure or NULL
 */
CmdCtrlNode *
wlan_get_cmd_node(wlan_private * priv)
{
    CmdCtrlNode *TempNode;
    wlan_adapter *Adapter = priv->adapter;
    ulong flags;

    ENTER();

    if (!Adapter)
        return NULL;

    spin_lock_irqsave(&Adapter->QueueSpinLock, flags);

    if (!list_empty(&Adapter->CmdFreeQ)) {
        TempNode = (CmdCtrlNode *) Adapter->CmdFreeQ.next;
        list_del((struct list_head *) TempNode);
    } else {
        PRINTM(WARN, "GET_CMD_NODE: CmdCtrlNode is not available\n");
        TempNode = NULL;
    }

    spin_unlock_irqrestore(&Adapter->QueueSpinLock, flags);

    LEAVE();
    return TempNode;
}

/** 
 *  @brief This function cleans command node.
 *  
 *  @param pTempNode	A pointer to CmdCtrlNode structure
 *  @return 		n/a
 */
void
wlan_clean_cmd_node(CmdCtrlNode * pTempNode)
{
    ENTER();

    if (!pTempNode)
        return;
    pTempNode->CmdWaitQWoken = TRUE;
    if (pTempNode->wait_option & HostCmd_OPTION_WAITFORRSP)
        wake_up_interruptible(&pTempNode->cmdwait_q);

    pTempNode->Status = 0;
    pTempNode->cmd_oid = (WLAN_OID) 0;
    pTempNode->wait_option = 0;
    pTempNode->CmdFlags = 0;
    pTempNode->pdata_buf = NULL;

    if (pTempNode->BufVirtualAddr != NULL)
        memset(pTempNode->BufVirtualAddr, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

    LEAVE();
    return;
}

/** 
 *  @brief This function initializes the command node.
 *  
 *  @param priv		A pointer to wlan_private structure
 *  @param pTempNode	A pointer to CmdCtrlNode structure
 *  @param cmd_oid	cmd oid: treated as sub command
 *  @param wait_option	wait option: wait response or not
 *  @param pdata_buf	A pointer to information buffer
 *  @return 		WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
void
wlan_init_cmd_node(wlan_private * priv,
                   CmdCtrlNode * pTempNode,
                   WLAN_OID cmd_oid, u16 wait_option, void *pdata_buf)
{
    ENTER();

    if (!pTempNode)
        return;

    pTempNode->cmd_oid = cmd_oid;
    pTempNode->wait_option = wait_option;
    pTempNode->pdata_buf = pdata_buf;

    LEAVE();
}

/** 
 *  @brief This function executes next command in command
 *  pending queue. It will put firmware back to PS mode
 *  if applicable.
 *  
 *  @param priv     A pointer to wlan_private structure
 *  @return 	   WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int
wlan_exec_next_cmd(wlan_private * priv)
{
    wlan_adapter *Adapter = priv->adapter;
    CmdCtrlNode *CmdNode = NULL;
    HostCmd_DS_COMMAND *CmdPtr;
    ulong flags;
    int ret = WLAN_STATUS_SUCCESS;

    ENTER();

    if (!Adapter) {
        PRINTM(MSG, "EXEC_NEXT_CMD: Adapter is NULL\n");
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    spin_lock_irqsave(&Adapter->QueueSpinLock, flags);

    if (Adapter->CurCmd) {
        PRINTM(MSG, "EXEC_NEXT_CMD: there is command in processing!\n");
        spin_unlock_irqrestore(&Adapter->QueueSpinLock, flags);
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    if (!list_empty(&Adapter->CmdPendingQ))
        CmdNode = (CmdCtrlNode *) Adapter->CmdPendingQ.next;

    spin_unlock_irqrestore(&Adapter->QueueSpinLock, flags);

    if (CmdNode) {
        CmdPtr = (HostCmd_DS_COMMAND *) CmdNode->BufVirtualAddr;

        if (wlan_is_cmd_allowed_in_ps(CmdPtr->Command)) {
            if ((Adapter->PSState == PS_STATE_SLEEP)
                || (Adapter->PSState == PS_STATE_PRE_SLEEP)
                ) {
                PRINTM(INFO,
                       "EXEC_NEXT_CMD: Cannot send cmd 0x%x in PSState %d\n",
                       CmdPtr->Command, Adapter->PSState);
                ret = WLAN_STATUS_FAILURE;
                goto done;
            }
            PRINTM(INFO, "EXEC_NEXT_CMD: OK to send command "
                   "0x%x in PSState %d\n", CmdPtr->Command, Adapter->PSState);
        } else if (Adapter->PSState != PS_STATE_FULL_POWER) {
            /* 
             * 1. Non-PS command: 
             * Queue it. set NeedToWakeup to TRUE if current state 
             * is SLEEP, otherwise call wlan_exit_ps to send Exit_PS.
             * 2. PS command but not Exit_PS:
             * Ignore it.
             * 3. PS command Exit_PS:
             * Set NeedToWakeup to TRUE if current state is SLEEP, 
             * otherwise send this command down to firmware
             * immediately.
             */
            if (CmdPtr->Command != wlan_cpu_to_le16(HostCmd_CMD_802_11_PS_MODE)) {
                /* Prepare to send Exit PS, this non PS command will be sent
                   later */
                if ((Adapter->PSState == PS_STATE_SLEEP)
                    || (Adapter->PSState == PS_STATE_PRE_SLEEP)
                    ) {
                    /* w/ new scheme, it will not reach here. since it is
                       blocked in main_thread. */
                    Adapter->NeedToWakeup = TRUE;
                } else
                    wlan_exit_ps(priv, 0);

                ret = WLAN_STATUS_SUCCESS;
                goto done;
            } else {
                /* 
                 * PS command. Ignore it if it is not Exit_PS. 
                 * otherwise send it down immediately.
                 */
                HostCmd_DS_802_11_PS_MODE *psm = &CmdPtr->params.psmode;

                PRINTM(INFO, "EXEC_NEXT_CMD: PS cmd- Action=0x%x\n",
                       psm->Action);
                if (psm->Action != wlan_cpu_to_le16(HostCmd_SubCmd_Exit_PS)) {
                    PRINTM(INFO, "EXEC_NEXT_CMD: Ignore Enter PS cmd\n");
                    list_del((struct list_head *) CmdNode);
                    wlan_insert_cmd_to_free_q(priv, CmdNode);

                    ret = WLAN_STATUS_SUCCESS;
                    goto done;
                }

                if ((Adapter->PSState == PS_STATE_SLEEP)
                    || (Adapter->PSState == PS_STATE_PRE_SLEEP)
                    ) {
                    PRINTM(INFO, "EXEC_NEXT_CMD: Ignore ExitPS cmd in sleep\n");
                    list_del((struct list_head *) CmdNode);
                    wlan_insert_cmd_to_free_q(priv, CmdNode);
                    Adapter->NeedToWakeup = TRUE;

                    ret = WLAN_STATUS_SUCCESS;
                    goto done;
                }

                PRINTM(INFO, "EXEC_NEXT_CMD: Sending Exit_PS down...\n");
            }
        }
        list_del((struct list_head *) CmdNode);
        wlan_dnld_cmd_to_fw(priv, CmdNode);
    } else {
        if (Adapter->MediaConnectStatus == WlanMediaStateConnected) {
            /* 
             * check if in power save mode, if yes, put the device back
             * to PS mode
             */
            if ((Adapter->PSMode != Wlan802_11PowerModeCAM) &&
                (Adapter->PSState == PS_STATE_FULL_POWER)) {
                if (Adapter->SecInfo.WPAEnabled || Adapter->SecInfo.WPA2Enabled) {
                    if (Adapter->IsGTK_SET) {
                        PRINTM(INFO, "EXEC_NEXT_CMD: WPA enabled and GTK_SET"
                               " go back to PS_SLEEP");
                        wlan_enter_ps(priv, 0);
                    }
                } else {
                    if ((Adapter->InfrastructureMode != Wlan802_11IBSS)
                        || Adapter->CurBssParams.BSSDescriptor.ATIMWindow) {
                        PRINTM(INFO, "EXEC_NEXT_CMD: Command PendQ is empty,"
                               " go back to PS_SLEEP");
                        wlan_enter_ps(priv, 0);
                    }
                }
            }
        } else {
            /* 
             * check if in auto deep sleep mode, if yes, put the device back
             * to DS mode
             */
            if (Adapter->IsAutoDeepSleepEnabled && !Adapter->IntCounter) {
                PRINTM(INFO, "Entering Auto Deep Sleep mode...\n");
                ret = wlan_prepare_cmd(priv,
                                       HostCmd_CMD_802_11_DEEP_SLEEP, 0,
                                       0, 0, NULL);
            }
        }
        /* The hs_activate command is sent when Host Sleep is configured and
           de-activated in full power mode. */
        if (Adapter->bHostSleepConfigured && !Adapter->HS_Activated
            && ((Adapter->MediaConnectStatus == WlanMediaStateConnected)
                || (!Adapter->IsAutoDeepSleepEnabled))
            && (((Adapter->PSMode == Wlan802_11PowerModeCAM)
                 && (Adapter->PSState == PS_STATE_FULL_POWER))
                || ((Adapter->InfrastructureMode == Wlan802_11IBSS)
                    && !Adapter->CurBssParams.BSSDescriptor.ATIMWindow)
            )) {
            ret = wlan_prepare_cmd(priv,
                                   HostCmd_CMD_802_11_HOST_SLEEP_ACTIVATE,
                                   0, 0, 0, NULL);
        }
    }

    ret = WLAN_STATUS_SUCCESS;
  done:
    LEAVE();
    return ret;
}

/** 
 *  @brief This function handles the timeout of command sending.
 *  It will re-send the same command again.
 *  
 *  @param FunctionContext    A pointer to FunctionContext
 *  @return 	   n/a
 */
void
wlan_cmd_timeout_func(void *FunctionContext)
{
    wlan_private *priv = (wlan_private *) FunctionContext;
    wlan_adapter *Adapter = priv->adapter;
    CmdCtrlNode *pTempNode;
    HostCmd_DS_COMMAND *CmdPtr;

    ENTER();

    Adapter->CommandTimerIsSet = FALSE;

    if (!Adapter->num_cmd_timeout)
        Adapter->dbg.num_cmd_timeout++;

    pTempNode = Adapter->CurCmd;

    if (pTempNode == NULL) {
        PRINTM(INFO, "CurCmd Empty\n");
        goto exit;
    }

    CmdPtr = (HostCmd_DS_COMMAND *) pTempNode->BufVirtualAddr;
    if (CmdPtr == NULL)
        goto exit;

    if (CmdPtr->Size) {
        Adapter->dbg.TimeoutCmdId = wlan_cpu_to_le16(CmdPtr->Command);
        Adapter->dbg.TimeoutCmdAct =
            wlan_cpu_to_le16(*(u16 *) ((u8 *) CmdPtr + S_DS_GEN));
        PRINTM(INFO, "Timeout cmd = 0x%x, act = 0x%x\n",
               Adapter->dbg.TimeoutCmdId, Adapter->dbg.TimeoutCmdAct);
    }
#define MAX_CMD_TIMEOUT_COUNT	5
    Adapter->num_cmd_timeout++;
    if (Adapter->num_cmd_timeout > MAX_CMD_TIMEOUT_COUNT) {
        Adapter->num_cmd_timeout = 0;
        PRINTM(FATAL, "Command timeout, cmd = 0x%x, act = 0x%x\n",
               Adapter->dbg.TimeoutCmdId, Adapter->dbg.TimeoutCmdAct);
        goto exit;
    }

    /* Restart the timer to trace command response again */
    wlan_mod_timer(&Adapter->MrvDrvCommandTimer, MRVDRV_TIMER_1S);
    Adapter->CommandTimerIsSet = TRUE;

    /* Wake up main thread to read int status register */
    Adapter->IntCounter++;
    wake_up_interruptible(&priv->MainThread.waitQ);

  exit:
    LEAVE();
    return;
}

/** 
 *  @brief This function sends sleep confirm command to firmware.
 *  
 *  @param priv    A pointer to wlan_private structure
 *  @param CmdPtr  A pointer to the command
 *  @param size	   the size of command
 *  @return 	   WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int
wlan_dnld_sleep_confirm_cmd(wlan_private * priv, u8 * CmdPtr, u16 size)
{
    wlan_adapter *Adapter = priv->adapter;
    int ret = WLAN_STATUS_SUCCESS;
    static u32 i = 0;

    ENTER();

    HEXDUMP("SLEEP_CFM", CmdPtr, size);

    ret = sbi_host_to_card(priv, MV_TYPE_CMD, CmdPtr, size);

    if (ret) {
        PRINTM(MSG, "SLEEP_CFM: sbi_host_to_card() failed\n");
        Adapter->dbg.num_cmd_sleep_cfm_host_to_card_failure++;
    } else {
        Adapter->PSState = PS_STATE_SLEEP;
        if (Adapter->IsEnhancedPSEnabled && !Adapter->sleep_period.period)
            Adapter->bWakeupDevRequired = TRUE;
        if (Adapter->bHostSleepConfigured &&
            (Adapter->sleep_period.period == 0)) {
            Adapter->bWakeupDevRequired = TRUE;
/*#define SDIO_CLK_300KHZ 300*/
            /* set the clock to the lowest frequency */
            /* sd_set_busclock((psd_device)priv->wlan_dev.card,
               SDIO_CLK_300KHZ); */
            wlan_host_sleep_activated_event(priv);
        }
#define NUM_SC_PER_LINE		16
        if (++i % NUM_SC_PER_LINE == 0)
            PRINTM(EVENT, "+\n");
        else
            PRINTM(EVENT, "+");

        /* check if interrupt is received after sleep confirm */
        if (Adapter->IntCounter) {
            PRINTM(INFO, "SLEEP_CFM: After sent, IntCnt=%d\n",
                   Adapter->IntCounter);
            Adapter->PSState = PS_STATE_AWAKE;
        }
    }

    LEAVE();
    return ret;
}

/** 
 *  @brief This function sends Enter_PS command to firmware.
 *  
 *  @param priv    	A pointer to wlan_private structure
 *  @param wait_option	wait response or not
 *  @return 	   	n/a 
 */
void
wlan_enter_ps(wlan_private * priv, int wait_option)
{
    wlan_adapter *Adapter = priv->adapter;
    u16 action;

    ENTER();

    if (Adapter->ieee_ps_mode == IEEE_PS_MODE_PSPOLL)
        action = HostCmd_SubCmd_Enter_PS_Poll;
    else
        action = HostCmd_SubCmd_Enter_PS;
    wlan_prepare_cmd(priv, HostCmd_CMD_802_11_PS_MODE,
                     action, wait_option, 0, NULL);

    LEAVE();
    return;
}

/** 
 *  @brief This function sends Eixt_PS command to firmware.
 *  
 *  @param priv    	A pointer to wlan_private structure
 *  @param wait_option	wait response or not
 *  @return 	   	n/a 
 */
void
wlan_exit_ps(wlan_private * priv, int wait_option)
{
    WLAN_802_11_POWER_MODE LocalPSMode;

    ENTER();

    LocalPSMode = Wlan802_11PowerModeCAM;

    wlan_prepare_cmd(priv, HostCmd_CMD_802_11_PS_MODE,
                     HostCmd_SubCmd_Exit_PS, wait_option, 0, &LocalPSMode);

    LEAVE();
    return;
}

/** 
 *  @brief This function checks condition and prepares to
 *  send sleep confirm command to firmware if OK.
 *  
 *  @param priv    	A pointer to wlan_private structure
 *  @param PSMode  	Power Saving mode
 *  @return 	   	n/a 
 */
void
wlan_ps_cond_check(wlan_private * priv, u16 PSMode)
{
    wlan_adapter *Adapter = priv->adapter;

    ENTER();

    if (!priv->wlan_dev.cmd_sent && !Adapter->CurCmd && !Adapter->IntCounter) {
        wlan_dnld_sleep_confirm_cmd(priv, (u8 *) & Adapter->PSConfirmSleep,
                                    sizeof(PS_CMD_ConfirmSleep));

        os_start_queue(priv);
    } else {
        PRINTM(INFO, "Delay Sleep Confirm (%s%s%s)\n",
               (priv->wlan_dev.cmd_sent) ? "D" : "",
               (Adapter->CurCmd) ? "C" : "", (Adapter->IntCounter) ? "I" : "");
    }

    LEAVE();
}
