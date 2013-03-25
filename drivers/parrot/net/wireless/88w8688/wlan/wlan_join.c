/** @file wlan_join.c
 *
 *  @brief Functions implementing wlan infrastructure and adhoc join routines
 *
 *  IOCTL handlers as well as command preparation and response routines
 *   for sending adhoc start, adhoc join, and association commands
 *   to the firmware.
 *  
 *  Copyright (C) 2003-2008, Marvell International Ltd. 
 *   
 *  This software file (the "File") is distributed by Marvell International 
 *  Ltd. under the terms of the GNU General Public License Version 2, June 1991 
 *  (the "License").  You may use, redistribute and/or modify this File in 
 *  accordance with the terms and conditions of the License, a copy of which 
 *  is available along with the File in the gpl.txt file or by writing to 
 *  the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 
 *  02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
 *  this warranty disclaimer.
 *
 */  
/*************************************************************
Change Log:
    01/11/06: Initial revision. Match new scan code. Relocate related functions
    01/19/06: Fix failure to save adhoc ssid as current after adhoc start
    03/16/06: Add a semaphore to protect reassociation thread

************************************************************/ 
    
#include    "wlan_headers.h"
    
/**
 *  @brief This function finds out the common rates between rate1 and rate2.
 *
 * It will fill common rates in rate1 as output if found.
 *
 * NOTE: Setting the MSB of the basic rates need to be taken
 *   care, either before or after calling this function
 *
 *  @param Adapter     A pointer to wlan_adapter structure
 *  @param rate1       the buffer which keeps input and output
 *  @param rate1_size  the size of rate1 buffer
 *  @param rate2       the buffer which keeps rate2
 *  @param rate2_size  the size of rate2 buffer.
 *
 *  @return            WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    static int
get_common_rates(wlan_adapter * Adapter, u8 * rate1, 
                 u8 * rate2, int rate2_size) 
{
    
    
    
    
    
    
        
        
        
    
    
    
    
        
            
                /* Check common rate, excluding the bit for basic rate */ 
                if ((rate2[i] & 0x7F) == (tmp[j] & 0x7F)) {
                
                
            
        
    
    
    
    
    
    
        
            
                
                
            
            
        
        
                "compatible with the network\n", Adapter->DataRate);
        
        
    
    
  
    
    



/**
 *  @brief Create the intersection of the rates supported by a target BSS and
 *         our Adapter settings for use in an assoc/join command.
 *
 *  @param Adapter       A pointer to wlan_adapter structure
 *  @param pBSSDesc      BSS Descriptor whose rates are used in the setup
 *  @param pOutRates     Output: Octet array of rates common between the BSS
 *                       and the Adapter supported rates settings
 *  @param pOutRatesSize Output: Number of rates/octets set in pOutRates
 *
 *  @return              WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 *
 */ 
    static int
setup_rates_from_bssdesc(wlan_adapter * Adapter, 
                         
{
    
    
    
    
        /* Copy AP supported rates */ 
        memcpy(pOutRates, pBSSDesc->SupportedRates, WLAN_SUPPORTED_RATES);
    
        /* Get the STA supported rates */ 
        switch (Adapter->config_bands)
        
    
        
        
        
        
    
        
        
        
        
    
    
    
        
        
        
        
    
        
        
        
        
    
        /* Get the common rates between AP and STA supported rates */ 
        if (get_common_rates
            (Adapter, pOutRates, WLAN_SUPPORTED_RATES, 
             card_rates_size)) {
        
        
        
        
    
    
    
    



/**
 *  @brief Convert band to radio type used in channel TLV
 *
 *  @param scanBand  Band enumeration to convert to a channel TLV radio type
 *
 *  @return          Radio type designator for use in a channel TLV
 *
 */ 
    static u8
band_to_radio_type(u8 band) 
{
    
    
    
        
        
    
    
    
        
        
    
    



/**
 *  @brief Retrieve the association response
 *
 *  @param priv         A pointer to wlan_private structure
 *  @param wrq          A pointer to iwreq structure
 *  @return             WLAN_STATUS_SUCCESS --success, otherwise fail
 */ 
    int
wlan_get_assoc_rsp_ioctl(wlan_private * priv, struct iwreq *wrq) 
{
    
    
    
    
        /* 
           the
           application
        copySize = MIN(Adapter->assocRspSize, wrq->u.data.length);
    
        /* Copy the (re)association response back to the application */ 
        if (copy_to_user
            (wrq->u.data.pointer, Adapter->assocRspBuffer, copySize)) {
        
        
        
    
    
        /* Returned copy length */ 
        wrq->u.data.length = copySize;
    
        /* Reset assoc buffer */ 
        Adapter->assocRspSize = 0;
    
        /* No error on return */ 
        LEAVE();
    



/**
 *  @brief Set an opaque block of Marvell TLVs for insertion into the
 *         association command
 *
 *  Pass an opaque block of data, expected to be Marvell TLVs, to the driver
 *    for eventual pass through to the firmware in an associate/join
 *    (and potentially start) command.
 *
 *
 *  @param priv         A pointer to wlan_private structure
 *  @param wrq          A pointer to iwreq structure
 *  @return             WLAN_STATUS_SUCCESS --success, otherwise fail
 */ 
    int
wlan_set_mrvl_tlv_ioctl(wlan_private * priv, struct iwreq *wrq) 
{
    
    
    
    
        /* If the passed length is zero, reset the buffer */ 
        if (wrq->u.data.length == 0)
        
    
    else {
        
            /* 
               available
            if (wrq->u.data.length < (sizeof(Adapter->mrvlAssocTlvBuffer) 
                                      -Adapter->mrvlAssocTlvBufferLen)) {
            
                /* Append the passed data to the end of the mrvlAssocTlvBuffer */ 
                if (copy_from_user(Adapter->mrvlAssocTlvBuffer 
                                   +Adapter->mrvlAssocTlvBufferLen,
                                   
                
                
                
            
            
                /* Increment the stored buffer length by the size passed */ 
                Adapter->mrvlAssocTlvBufferLen += wrq->u.data.length;
        
            
                /* Passed data does not fit in the remaining buffer space */ 
                ret = WLAN_STATUS_FAILURE;
        
    
    
        /* Return WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE */ 
        LEAVE();
    



/**
 *  @brief Stop Adhoc Network
 *
 *  @param priv         A pointer to wlan_private structure
 *  @return             WLAN_STATUS_SUCCESS --success, otherwise fail
 */ 
    int
wlan_do_adhocstop_ioctl(wlan_private * priv) 
{
    
    
    
    
        
    
    else {
        
        
    
    
    



/**
 *  @brief Set essid
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param dwrq         A pointer to iw_point structure
 *  @param extra        A pointer to extra data buf
 *  @return             WLAN_STATUS_SUCCESS--success, otherwise--fail
 */ 
    int
wlan_set_essid(struct net_device *dev, struct iw_request_info *info,
               
{
    
    
    
    
    
    
    
        
        
        
    
    
        /* Clear any past association response stored for application retrieval 
         */ 
        Adapter->assocRspSize = 0;
    
#ifdef REASSOCIATION
        /* cancel re-association timer if there's one */ 
        if (Adapter->ReassocTimerIsSet == TRUE) {
        
        
    
    
        
        
        
    
    
#endif  /* REASSOCIATION */
        
        /* Check the size of the string */ 
        if (dwrq->length > IW_ESSID_MAX_SIZE + 1) {
        
        
    
    
    
        /* 
        if (!dwrq->flags) {
        
            
            
            
        
    
        
            /* Set the SSID */ 
#if WIRELESS_EXT > 20
            reqSSID.SsidLength = dwrq->length;
        
#else   /* 
            reqSSID.SsidLength = dwrq->length - 1;
        
#endif  /* 
            memcpy(reqSSID.Ssid, extra,
                   MIN(reqSSID.SsidLength, MRVDRV_MAX_SSID_LENGTH));
    
    
             (reqSSID.SsidLength > 0) ? (char *) reqSSID.Ssid : "NULL");
    
        
        
        
    
    
        
            /* infrastructure mode */ 
            PRINTM(INFO, "SSID requested = %s\n", reqSSID.Ssid);
        
            
            
            
                
                    /* Failed to find in table since index is > current max. */ 
                    i = -EINVAL;
        
            
            
                                        
        
        
            
            
            
                
        
            
                                   wlan_find_ssid_in_list */
            
        
    
        
            /* ad hoc mode */ 
            /* If the requested SSID matches current SSID return */ 
            if (!wlan_ssid_cmp
                (&Adapter->CurBssParams.BSSDescriptor.Ssid, &reqSSID)) {
            
            
        
        
            /* 
            PRINTM(INFO, "Sending Adhoc Stop\n");
        
        
            
        
        
            
            
                
                    /* Failed to find in table since index is > current max. */ 
                    i = -EINVAL;
        
            
                /* Scan for the network */ 
                wlan_cmd_specific_scan_ssid(priv, &reqSSID);
            
                /* Search for the requested SSID in the scan table */ 
                i =
                wlan_find_ssid_in_list(Adapter, &reqSSID, NULL, Wlan802_11IBSS);
        
        
            
            
        
            
                /* else send START command */ 
                PRINTM(INFO, "SSID not found in list, " 
                       "so creating adhoc with ssid = %s\n", 
            
        
    
    
        /* 
           code is being properly returned.
        /* Check to see if we successfully connected */ 
        if (Adapter->MediaConnectStatus == WlanMediaStateConnected)
        
    
    else
        
  
#ifdef REASSOCIATION
        OS_REL_SEMAPHORE(&Adapter->ReassocSem);
    
#endif  /* 
        
    



/**
 *  @brief Connect to the AP or Ad-hoc Network with specific bssid
 *
 * NOTE: Scan should be issued by application before this function is called
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param awrq         A pointer to iw_param structure
 *  @param extra        A pointer to extra data buf
 *  @return             WLAN_STATUS_SUCCESS --success, otherwise fail
 */ 
    int
wlan_set_wap(struct net_device *dev, struct iw_request_info *info,
             
{
    
    
    
    
    
    
    
    
    
        
        
        
    
    
        
        
    
    
        /* Clear any past association response stored for application retrieval 
         */ 
        Adapter->assocRspSize = 0;
    
            (u8) awrq->sa_data[0], (u8) awrq->sa_data[1], 
            (u8) awrq->sa_data[2], (u8) awrq->sa_data[3], 
            (u8) awrq->sa_data[4], (u8) awrq->sa_data[5]);
    
#ifdef REASSOCIATION
        /* cancel re-association timer if there's one */ 
        if (Adapter->ReassocTimerIsSet == TRUE) {
        
        
    
    
#endif  /* REASSOCIATION */
        /* zeroMac mean disconnect */ 
        if (!memcmp(zeroMac, awrq->sa_data, ETH_ALEN)) {
        
        
    
    
        
    
    else {
        
            /* check if we already assoicate to the AP */ 
            if (Adapter->MediaConnectStatus == WlanMediaStateConnected) {
            
                 (awrq->sa_data, Adapter->CurBssParams.BSSDescriptor.MacAddress,
                  
                
        
        
        
                 
                 reqBSSID[4], reqBSSID[5]);
        
            /* Search for index position in list for requested MAC */ 
            i =
            wlan_find_bssid_in_list(Adapter, reqBSSID,
                                    Adapter->InfrastructureMode);
    
    
        
        
        
    
    
        
            
    
        
            /* Exit Adhoc mode */ 
            if ((ret = wlan_disconnect(priv)))
            
        
        
    
    
        /* Check to see if we successfully connected */ 
        if (Adapter->MediaConnectStatus == WlanMediaStateConnected)
        
    
    else
        
  
    



/**
 *  @brief Associated to a specific BSS discovered in a scan
 *
 *  @param priv      A pointer to wlan_private structure
 *  @param pBSSDesc  Pointer to the BSS descriptor to associate with.
 *
 *  @return          WLAN_STATUS_SUCCESS-success, otherwise fail
 */ 
    int
wlan_associate(wlan_private * priv, BSSDescriptor_t * pBSSDesc) 
{
    
    
    
    
    
    
    
    
    
        /* Return error if the Adapter or table entry is not marked as infra */ 
        if ((Adapter->InfrastructureMode != Wlan802_11Infrastructure) 
            ||(pBSSDesc->InfrastructureMode != Wlan802_11Infrastructure)) {
        
        
    
    
             
    
        
        
        
    
    
        /* Clear any past association response stored for application retrieval 
         */ 
        Adapter->assocRspSize = 0;
    
        wlan_prepare_cmd(priv, HostCmd_CMD_802_11_ASSOCIATE, 
                         HostCmd_OPTION_WAITFORRSP, 
    
    
    /** status code: Responding station doesn't support the specific auth */ 
#define ERROR_STA_NOT_SUPPORT_SPECIFIC_AUTH 13
        if ((Adapter->MediaConnectStatus != WlanMediaStateConnected) && 
            (Adapter->AuthType == AUTHTYPE_ALLOW_BOTH) && 
            (Adapter->SecInfo.WEPStatus == Wlan802_11WEPEnabled) &&
            Adapter->assocRspSize &&
            
        
            
        
        else
            
        
        
            wlan_prepare_cmd(priv, HostCmd_CMD_802_11_ASSOCIATE, 
                             HostCmd_OPTION_WAITFORRSP, 
    
    
        
            /* Don't re-enable carrier until we get the WMM_GET_STATUS event */ 
            enableData = FALSE;
    
    else
        
            /* Since WMM is not enabled, setup the queues with the defaults */ 
            wmm_setup_queues(priv);
    
        
              &&(memcmp
                 (&currentBSSID,
                  
                  
            
                /* Reassociation attempt failed, still associated to old AP,
                   no need to wait for WMM notification to restart data */ 
                enableData = TRUE;
        
            
            
            
            
        
    
    
    else {
        
        
        
    
    
        
    
    else {
        
                
        
        
    
    
    



/**
 *  @brief Associated to a specific indexed entry in the ScanTable
 *
 *  @param priv      A pointer to wlan_private structure
 *  @param tableIdx  Index into the ScanTable to associate to, index parameter
 *                   base value is 1.  No scanning is done before the 
 *                   association attempt.
 *
 *  @return          WLAN_STATUS_SUCCESS-success, otherwise fail
 */ 
    int
wlan_associate_to_table_idx(wlan_private * priv, int tableIdx) 
{
    
    
    
    
#ifdef REASSOCIATION
        if (OS_ACQ_SEMAPHORE_BLOCK(&Adapter->ReassocSem)) {
        
        
    
    
#endif  /* 
        
                 
    
        /* 
           association
           based to
           iwconfig/iwlist
        if (tableIdx && (tableIdx <= Adapter->NumInScanTable))
        
    
    else
        
    
#ifdef REASSOCIATION
        OS_REL_SEMAPHORE(&Adapter->ReassocSem);
    
#endif  /* 
        LEAVE();
    



/**
 *  @brief Start an Adhoc Network
 *
 *  @param priv         A pointer to wlan_private structure
 *  @param AdhocSSID    The ssid of the Adhoc Network
 *  @return             WLAN_STATUS_SUCCESS--success, WLAN_STATUS_FAILURE--fail
 */ 
    int
wlan_start_adhoc(wlan_private * priv, WLAN_802_11_SSID * AdhocSSID) 
{
    
    
    
    
        
        
            /* 
               availability
            if (wlan_11h_radar_detect_required(priv, Adapter->AdhocChannel)) {
            
                /* 
                   11h is activated in the firmware
                wlan_11h_activate(priv, TRUE);
            
                /* Check for radar on the channel */ 
                if (wlan_11h_radar_detected(priv, Adapter->AdhocChannel)) {
                
                
            
        
    
    
    
            
    
    
        wlan_prepare_cmd(priv, HostCmd_CMD_802_11_AD_HOC_START, 
                         HostCmd_OPTION_WAITFORRSP, 
    
    



/**
 *  @brief Join an adhoc network found in a previous scan
 *
 *  @param priv         A pointer to wlan_private structure
 *  @param pBSSDesc     Pointer to a BSS descriptor found in a previous scan
 *                      to attempt to join
 *
 *  @return             WLAN_STATUS_SUCCESS--success, WLAN_STATUS_FAILURE--fail
 */ 
    int
wlan_join_adhoc(wlan_private * priv, BSSDescriptor_t * pBSSDesc) 
{
    
    
    
    
             
    
            
    
    
            
    
        /* check if the requested SSID is already joined */ 
        if (Adapter->CurBssParams.BSSDescriptor.Ssid.SsidLength 
            &&!wlan_ssid_cmp(&pBSSDesc->Ssid,
                             
                             Ssid) 
                                        InfrastructureMode ==
                                        
        
                 
                 "not attempting to re-join");
        
        
    
    
             
    
    
        wlan_prepare_cmd(priv, HostCmd_CMD_802_11_AD_HOC_JOIN, 
                         HostCmd_OPTION_WAITFORRSP, 
    
    



/**
 *  @brief Send Deauthentication Request or Stop the AdHoc network depending on mode
 *
 *  @param priv      A pointer to wlan_private structure
 *  @return          WLAN_STATUS_SUCCESS--success, WLAN_STATUS_FAILURE--fail
 */ 
    int
wlan_disconnect(wlan_private * priv) 
{
    
    
    
    
    
        
            
                    (void *) &Adapter->CurBssParams.BSSDescriptor.MacAddress,
                    
            
                wlan_prepare_cmd(priv, HostCmd_CMD_802_11_DEAUTHENTICATE, 
                                 HostCmd_OPTION_WAITFORRSP |
                                 HostCmd_OPTION_TIMEOUT, 
        
            
                wlan_prepare_cmd(priv, HostCmd_CMD_802_11_AD_HOC_STOP, 
                                 HostCmd_OPTION_WAITFORRSP |
                                 HostCmd_OPTION_TIMEOUT, 
        
    
    
    



/**
 *  @brief Set Idle Off
 *
 *  @param priv         A pointer to wlan_private structure
 *  @return             WLAN_STATUS_SUCCESS --success, otherwise fail
 */ 
    int
wlanidle_off(wlan_private * priv) 
{
    
    
    
    
    
    
        
            
                
                         
                
                        "Previous BSSID = " 
                        "%02x:%02x:%02x:%02x:%02x:%02x:\n",
                        
                        
                        
                
                    wlan_find_ssid_in_list(Adapter, 
                                           
                                           
                
                    
                    
                                                
                                                
                                                
                
                
                    
                        /* If the BSSID could not be found, try just the SSID */ 
                        i =
                        wlan_find_ssid_in_list(Adapter, 
                                               
                                               
                
                    
                    
                                                
                                                
                
                
                    
            
        
            
                wlan_prepare_cmd(priv, 
                                 HostCmd_OPTION_WAITFORRSP, 
                                 &Adapter->PreviousSSID);
        
    
    
        /* else it is connected */ 
        
    
    



/**
 *  @brief Set Idle On
 *
 *  @param priv         A pointer to wlan_private structure
 *  @return             WLAN_STATUS_SUCCESS --success, otherwise fail
 */ 
    int
wlanidle_on(wlan_private * priv) 
{
    
    
    
    
        
            
            
                    
                    
            
        
            
    
    
#ifdef REASSOCIATION
        if (Adapter->ReassocTimerIsSet == TRUE) {
        
        
    
    
#endif  /* REASSOCIATION */
        
    
    



/**
 *  @brief Append a generic IE as a pass through TLV to a TLV buffer.
 *
 *  This function is called from the network join command prep. routine. 
 *    If the IE buffer has been setup by the application, this routine appends
 *    the buffer as a pass through TLV type to the request.
 *
 *  @param priv     A pointer to wlan_private structure
 *  @param ppBuffer pointer to command buffer pointer
 *
 *  @return         bytes added to the buffer
 */ 
    static int
wlan_cmd_append_generic_ie(wlan_private * priv, u8 ** ppBuffer) 
{
    
    
    
    
        /* Null Checks */ 
        if (ppBuffer == 0)
        return 0;
    
        return 0;
    
        /* 
           parameter buffer pointer.
        if (Adapter->genIeBufferLen) {
        
                
        
            /* Wrap the generic IE buffer with a pass through TLV type */ 
            ieHeader.Type = wlan_cpu_to_le16(TLV_TYPE_PASSTHROUGH);
        
        
        
            /* Increment the return size and the return buffer pointer param */ 
            *ppBuffer += sizeof(ieHeader);
        
        
            /* Copy the generic IE buffer to the output buffer, advance pointer 
             */ 
            memcpy(*ppBuffer, Adapter->genIeBuffer, Adapter->genIeBufferLen);
        
            /* Increment the return size and the return buffer pointer param */ 
            *ppBuffer += Adapter->genIeBufferLen;
        
        
            /* Reset the generic IE buffer */ 
            Adapter->genIeBufferLen = 0;
    
    
        /* return the length appended to the buffer */ 
        return retLen;



/**
 *  @brief Append any application provided Marvell TLVs to a TLV buffer.
 *
 *  This function is called from the network join command prep. routine. 
 *    If the Marvell TLV buffer has been setup by the application, this routine
 *    appends the buffer to the request.
 *
 *  @param priv     A pointer to wlan_private structure
 *  @param ppBuffer pointer to command buffer pointer
 *
 *  @return         bytes added to the buffer
 */ 
    static int
wlan_cmd_append_marvell_tlv(wlan_private * priv, u8 ** ppBuffer) 
{
    
    
    
        /* Null Checks */ 
        if (ppBuffer == 0)
        return 0;
    
        return 0;
    
        /* 
           parameter buffer pointer.
        if (Adapter->mrvlAssocTlvBufferLen) {
        
                
        
            /* Copy the TLV buffer to the output buffer, advance pointer */ 
            memcpy(*ppBuffer, 
                   
        
            /* Increment the return size and the return buffer pointer param */ 
            *ppBuffer += Adapter->mrvlAssocTlvBufferLen;
        
        
            /* Reset the Marvell TLV buffer */ 
            Adapter->mrvlAssocTlvBufferLen = 0;
    
    
        /* return the length appended to the buffer */ 
        return retLen;



/**
 *  @brief Append TSF tracking info from the scan table for the target AP
 *
 *  This function is called from the network join command prep. routine. 
 *    The TSF table TSF sent to the firmware contains two TSF values:
 *      - the TSF of the target AP from its previous beacon/probe response
 *      - the TSF timestamp of our local MAC at the time we observed the
 *        beacon/probe response.
 *
 *    The firmware uses the timestamp values to set an initial TSF value
 *      in the MAC for the new association after a reassociation attempt.
 *
 *  @param priv     A pointer to wlan_private structure
 *  @param ppBuffer A pointer to command buffer pointer
 *  @param pBSSDesc A pointer to the BSS Descriptor from the scan table of
 *                  the AP we are trying to join
 *
 *  @return         bytes added to the buffer
 */ 
    static int
wlan_cmd_append_tsf_tlv(wlan_private * priv, u8 ** ppBuffer,
                        
{
    
    
    
        /* Null Checks */ 
        if (ppBuffer == 0)
        return 0;
    
        return 0;
    
    
    
    
    
    
        /* TSF timestamp from the firmware TSF when the bcn/prb rsp was
           received */ 
        tsfVal = wlan_cpu_to_le64(pBSSDesc->networkTSF);
    
    
    
    
             pBSSDesc->networkTSF);
    
    
    
    



/**
 *  @brief This function prepares command of deauthenticate.
 *
 *  @param priv     A pointer to wlan_private structure
 *  @param cmd      A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf Void cast of MAC Address to deauth
 *  @return         WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    int
wlan_cmd_802_11_deauthenticate(wlan_private * priv, 
                               
{
    
    
    
    
    
        
    
        /* set AP MAC address */ 
        memcpy(pDeauth->MacAddr, p_saddr->sa_data, ETH_ALEN);
    
        /* Reason code 3 = Station is leaving */ 
#define REASON_CODE_STA_LEAVING 3
        pDeauth->ReasonCode = wlan_cpu_to_le16(REASON_CODE_STA_LEAVING);
    
    



/**
 *  @brief This function prepares command of association.
 *
 *  @param priv      A pointer to wlan_private structure
 *  @param cmd       A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf Void cast of BSSDescriptor_t from the scan table to assoc
 *  @return          WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    int
wlan_cmd_802_11_associate(wlan_private * priv, 
                          
{
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
        /* Save so we know which BSS Desc to use in the response handler */ 
        Adapter->pAttemptedBSSDesc = pBSSDesc;
    
             
    
    
        /* set the listen interval */ 
        pAsso->ListenInterval = wlan_cpu_to_le16(Adapter->ListenInterval);
    
    
    
    
    
    
    
    
    
    
    
    
    
            
            
    
    
    
    
    
    
    
    
        /* Get the common rates supported between the driver and the BSS Desc */ 
        if (setup_rates_from_bssdesc(Adapter, pBSSDesc, rates, &ratesSize)) {
        
        
    
    
        /* Setup the Rates TLV in the association command */ 
        pRatesTlv = (MrvlIEtypes_RatesParamSet_t *) pos;
    
    
    
    
    
    
        /* Add the Authentication type to be used for Auth frames if needed */ 
        pAuthTlv = (MrvlIEtypes_AuthType_t *) pos;
    
    
    
    
    
    
        /* Append a channel TLV for the channel the attempted AP was found on */ 
        pChanTlv = (MrvlIEtypes_ChanListParamSet_t *) pos;
    
    
    
    
        (pBSSDesc->PhyParamSet 
    
            
    
        band_to_radio_type(pBSSDesc->bss_band);
    
             
    
    
        
        
            
            
            
                                                                   WPA2_IE */
            
            
            
            
            
                
                        pRsnTlv->Header.Len);
            
            else {
                
                
            
            
                     
            
            
            
        
    
    
    
    
    
        
        
        
    
    
        
        
    
    
        /* 
           11h
           appended
        wlan_11h_process_join(priv, &pos, &pAsso->CapInfo,
                              
                              
    
    
        /* set the Capability info at last */ 
        memcpy(&TmpCap, &pBSSDesc->Cap, sizeof(pAsso->CapInfo));
    
    
            CAPINFO_MASK);
    
    
  
    



/**
 *  @brief This function prepares command of ad_hoc_start.
 *
 *  @param priv     A pointer to wlan_private structure
 *  @param cmd      A pointer to HostCmd_DS_COMMAND structure
 *  @param pssid    A pointer to WLAN_802_11_SSID structure
 *  @return         WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    int
wlan_cmd_802_11_ad_hoc_start(wlan_private * priv, 
                             
{
    
    
    
    
    
    
    
    
    
    
    
    
    
        
        
    
    
    
    
    
        /* 
           HostCmd_DS_802_11_AD_HOC_START Command
           fill up SSID, BSSType,IBSS param, Physical Param,
           Cap info.
           operational rates.
        
    
             ((WLAN_802_11_SSID *) pssid)->SsidLength);
    
    
    
            ((WLAN_802_11_SSID *) pssid)->Ssid, 
            ((WLAN_802_11_SSID *) pssid)->SsidLength);
    
    
        /* set the BSS type */ 
        adhs->BSSType = HostCmd_BSS_TYPE_IBSS;
    
    
    
    
        /* set Physical param set */ 
#define DS_PARA_IE_ID   3
#define DS_PARA_IE_LEN  1
        
    
    
          (Adapter->adhoc_start_band, (u16) Adapter->AdhocChannel,
           Adapter->region_channel)) {
        
        
            get_cfp_by_band_and_channel(Adapter->adhoc_start_band,
                                        
                                        Adapter->region_channel);
        
            
    
    
    
             
    
    
    
    
    
             sizeof(IEEEtypes_PhyParamSet_t));
    
    
        /* set IBSS param set */ 
#define IBSS_PARA_IE_ID   6
#define IBSS_PARA_IE_LEN  2
        
    
    
        = wlan_cpu_to_le16(Adapter->AtimWindow);
    
    
            
    
        /* set Capability info */ 
        adhs->Cap.Ess = 0;
    
    
    
        /* set up privacy in pBSSDesc */ 
        if (Adapter->SecInfo.WEPStatus == Wlan802_11WEPEnabled 
            ||Adapter->AdhocAESEnabled 
        
#define AD_HOC_CAP_PRIVACY_ON 1
            PRINTM(INFO, "ADHOC_S_CMD: WEPStatus set, Privacy to WEP\n");
        
        
    
        
                "Privacy to ACCEPT ALL\n");
        
    
    
    
        
                
    
                
        
                
        
            wlan_prepare_cmd(priv, 
                             HostCmd_OPTION_WAITFORRSP, 
                             &Adapter->CurrentPacketFilter);
        
            
            
            
        
    
        
                
    
        /* Find the last non zero */ 
        for (i = 0; i < sizeof(adhs->DataRate) && adhs->DataRate[i]; i++)
        
    
    
        /* Copy the ad-hoc creating rates into Current BSS state structure */ 
        memcpy(&Adapter->CurBssParams.DataRates, 
               
    
             
             adhs->DataRate[3]);
    
    
        
            /* Append a channel TLV */ 
            pChanTlv = (MrvlIEtypes_ChanListParamSet_t *) pos;
        
        
        
        
            Adapter->CurBssParams.BSSDescriptor.Channel;
        
                 
        
            band_to_radio_type(Adapter->CurBssParams.band);
        
                 
        
        
    
    
        
        
        
    
    
        /* 
           parameters
        cmdAppendSize +=
        wlan_11h_process_start(priv, &pos, &adhs->Cap, 
                               
    
        
        
                                                                   WPA2_IE */
        
        
        
        
        
            
        
        else {
            
            
        
        
                  
        
        
        
    
    
                                    +S_DS_GEN + cmdAppendSize);
    
    
    
    
  
    



/**
 *  @brief This function prepares command of ad_hoc_stop.
 *
 *  @param priv     A pointer to wlan_private structure
 *  @param cmd      A pointer to HostCmd_DS_COMMAND structure
 *  @return         WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    int
wlan_cmd_802_11_ad_hoc_stop(wlan_private * priv, 
{
    
    
    
                                  +S_DS_GEN);
    
        
    
    



/**
 *  @brief This function prepares command of ad_hoc_join.
 *
 *  @param priv      A pointer to wlan_private structure
 *  @param cmd       A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf Void cast of BSSDescriptor_t from the scan table to join
 *
 *  @return          WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    int
wlan_cmd_802_11_ad_hoc_join(wlan_private * priv, 
                            
{
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
#define USE_G_PROTECTION	0x02    
        if (pBSSDesc->ERPFlags & USE_G_PROTECTION) {
        
            Adapter->
            CurrentPacketFilter | HostCmd_ACT_MAC_ADHOC_G_PROTECTION_ON;
        
            wlan_prepare_cmd(priv, 
                             HostCmd_OPTION_WAITFORRSP, 
                             &CurrentPacketFilter);
        
            
            
            
        
    
    
    
    
    
        = wlan_cpu_to_le16(pBSSDesc->BeaconPeriod);
    
             
    
             
    
             
    
             
    
    
    
             CAPINFO_MASK);
    
            
    
        /* information on BSSID descriptor passed to FW */ 
        PRINTM(INFO,
               
               
               
               
               
               
               
               
    
        /* Get the common rates supported between the driver and the BSS Desc */ 
        if (setup_rates_from_bssdesc(Adapter, pBSSDesc, rates, &ratesSize)) {
        
        
    
    
        /* Copy Data Rates from the Rates recorded in scan response */ 
        memset(pAdHocJoin->BssDescriptor.DataRates, 0,
               
    
    
        /* Copy the adhoc join rates into Current BSS state structure */ 
        Adapter->CurBssParams.NumOfRates = ratesSize;
    
    
        /* Copy the channel information */ 
        Adapter->CurBssParams.BSSDescriptor.Channel = pBSSDesc->Channel;
    
    
          ||Adapter->AdhocAESEnabled 
        
    
        
            /* wake up first */ 
            WLAN_802_11_POWER_MODE LocalPSMode;
        
        
            wlan_prepare_cmd(priv, 
                             
        
            
            
        
    
    
        
            /* Append a channel TLV */ 
            pChanTlv = (MrvlIEtypes_ChanListParamSet_t *) pos;
        
        
        
        
            (pBSSDesc->PhyParamSet 
        
                
        
            band_to_radio_type(pBSSDesc->bss_band);
        
                 
        
        
    
    
        
        
        
    
    
        
        
    
    
        /* 
           11h behavior can be properly triggered.
           appended
        cmdAppendSize +=
        wlan_11h_process_join(priv, &pos, 
                              
                              
    
        
        
                                                                   WPA2_IE */
        
        
        
        
        
            
        
        else {
            
            
        
        
                  
        
        
        
    
    
                                   +S_DS_GEN + cmdAppendSize);
    
             
    
    
             
  
    



/**
 *  @brief Association firmware command response handler
 *
 *   The response buffer for the association command has the following
 *      memory layout.
 *
 *   For cases where an association response was not received (indicated
 *      by the CapInfo and AId field):
 *
 *     .------------------------------------------------------------.
 *     |  Header(4 * sizeof(u16)):  Standard command response hdr   |
 *     .------------------------------------------------------------.
 *     |  CapInfo/Error Return(u16):                                |
 *     |           0xFFFF(-1): Internal error                       |
 *     |           0xFFFE(-2): Authentication unhandled message     |
 *     |           0xFFFD(-3): Authentication refused               |
 *     |           0xFFFC(-4): Timeout waiting for AP response      |
 *     .------------------------------------------------------------.
 *     |  StatusCode(u16):                                          |
 *     |        If CapInfo is -1:                                   |
 *     |           An internal firmware failure prevented the       |
 *     |           command from being processed.  The StatusCode    |
 *     |           will be set to 1.                                |
 *     |                                                            |
 *     |        If CapInfo is -2:                                   |
 *     |           An authentication frame was received but was     |
 *     |           not handled by the firmware.  IEEE Status        |
 *     |           code for the failure is returned.                |
 *     |                                                            |
 *     |        If CapInfo is -3:                                   |
 *     |           An authentication frame was received and the     |
 *     |           StatusCode is the IEEE Status reported in the    |
 *     |           response.                                        |
 *     |                                                            |
 *     |        If CapInfo is -4:                                   |
 *     |           (1) Association response timeout                 |
 *     |           (2) Authentication response timeout              |
 *     .------------------------------------------------------------.
 *     |  AId(u16): 0xFFFF                                          |
 *     .------------------------------------------------------------.
 *
 *
 *   For cases where an association response was received, the IEEE 
 *     standard association response frame is returned:
 *
 *     .------------------------------------------------------------.
 *     |  Header(4 * sizeof(u16)):  Standard command response hdr   |
 *     .------------------------------------------------------------.
 *     |  CapInfo(u16): IEEE Capability                             |
 *     .------------------------------------------------------------.
 *     |  StatusCode(u16): IEEE Status Code                         |
 *     .------------------------------------------------------------.
 *     |  AId(u16): IEEE Association ID                             |
 *     .------------------------------------------------------------.
 *     |  IEEE IEs(variable): Any received IEs comprising the       |
 *     |                      remaining portion of a received       |
 *     |                      association response frame.           |
 *     .------------------------------------------------------------.
 *
 *  For simplistic handling, the StatusCode field can be used to determine
 *    an association success (0) or failure (non-zero).
 *
 *  @param priv    A pointer to wlan_private structure
 *  @param resp    A pointer to HostCmd_DS_COMMAND
 *  @return        WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    int
wlan_ret_802_11_associate(wlan_private * priv, 
{
    
    
    
    
    
    
    
    
    
              
    
        MIN(wlan_le16_to_cpu(resp->Size) - S_DS_GEN,
            
    
    
        
        
                "status code = %d, error = %d\n", 
                *(short *) &pAssocRsp->Capability);
        
        
    
    
        /* Send a Media Connected event, according to the Spec */ 
        Adapter->MediaConnectStatus = WlanMediaStateConnected;
    
        /* Set the attempted BSSID Index to current */ 
        pBSSDesc = Adapter->pAttemptedBSSDesc;
    
    
        /* Make a copy of current BSSID descriptor */ 
        memcpy(&Adapter->CurBssParams.BSSDescriptor, 
               
    
        /* update CurBssParams */ 
        Adapter->CurBssParams.BSSDescriptor.Channel 
        = pBSSDesc->PhyParamSet.DsParamSet.CurrentChan;
    
    
        
        
    
    
        /* Copy the infra. association rates into Current BSS state structure */ 
        Adapter->CurBssParams.NumOfRates = ratesSize;
    
    
        /* Adjust the timestamps in the scan table to be relative to the newly
           associated AP's TSF
        wlan_scan_update_tsf_timestamps(priv, pBSSDesc);
    
        
    
    else
        
    
        
    
    else
        
    
    
        
            = pBSSDesc->wmmIE.QoSInfo.QosUAPSD;
    
             
    
        
    
    
    
    
    
    
    
    
    
    
  
    



/**
 *  @brief This function handles the command response of deauthenticate
 *
 *  @param priv    A pointer to wlan_private structure
 *  @param resp    A pointer to HostCmd_DS_COMMAND
 *  @return        WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    int
wlan_ret_802_11_deauthenticate(wlan_private * priv,
                               
{
    
    
    
    
         (resp->params.deauth.MacAddr,
          
          
        
    
    



/**
 *  @brief This function handles the command response of ad_hoc_start and
 *  ad_hoc_join
 *
 *  @param priv    A pointer to wlan_private structure
 *  @param resp    A pointer to HostCmd_DS_COMMAND
 *  @return        WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    int
wlan_ret_802_11_ad_hoc(wlan_private * priv, 
{
    
    
    
    
    
    
    
    
    
    
    
        /* 
        if (Result) {
        
        
            
        
                 
        
        
    
    
        /* Send a Media Connected event, according to the Spec */ 
        Adapter->MediaConnectStatus = WlanMediaStateConnected;
    
        
        
            /* Update the created network descriptor with the new BSSID */ 
            memcpy(pBSSDesc->MacAddress, 
        
    
        
            /* 
               use SSID to compare instead of BSSID
            PRINTM(INFO, "ADHOC_J_RESP  %s\n", pBSSDesc->Ssid.Ssid);
        
            /* Make a copy of current BSSID descriptor, only needed for join
               since
               start */ 
            memcpy(&Adapter->CurBssParams.BSSDescriptor, 
                   
        
    
    
    
            
    
    
    
    
            
            
            
            
            
            
    
    



/**
 *  @brief This function handles the command response of ad_hoc_stop
 *
 *  @param priv    A pointer to wlan_private structure
 *  @param resp    A pointer to HostCmd_DS_COMMAND
 *  @return        WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */ 
    int
wlan_ret_802_11_ad_hoc_stop(wlan_private * priv, 
{
    
    
    
    



#ifdef REASSOCIATION
/**
 *  @brief This function handles re-association. it is triggered
 *  by re-assoc timer.
 *
 *  @param data    A pointer to wlan_thread structure
 *  @return        WLAN_STATUS_SUCCESS
 */ 
    int
wlan_reassociation_thread(void *data) 
{
    
    
    
    
    
    
    
    
    
    
    
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
        current->flags |= PF_NOFREEZE;
    
#endif  /* 
        
        
        
        
        
        
        
        
            
        
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
            if (thread->state == WLAN_THREAD_STOPPED)
            
        
#else   /* 
            if (kthread_should_stop())
            
        
#endif  /* 
            
        
              
            
            
        
        
            /* The semaphore is used to avoid reassociation thread and 
               wlan_set_scan/wlan_set_essid interrupting each other.
               Reassociation should be disabled completely by application if 
               wlan_set_user_scan_ioctl/wlan_set_wap is used.
            if (OS_ACQ_SEMAPHORE_BLOCK(&Adapter->ReassocSem)) {
            
            
        
        
            
            
            
        
        
                 
        
                 
        
        
            
            
        
        
            wlan_find_ssid_in_list(Adapter, 
                                   
                                   
        
            
                /* If the SSID could not be found, try just the SSID */ 
                i =
                wlan_find_ssid_in_list(Adapter, 
                                       
        
        
            
                
                    wlan_prepare_cmd(priv, 
                                     
                                     HostCmd_OPTION_WAITFORRSP, 
                
                    
            
            
        
        
      
             WlanMediaStateDisconnected) {
            
                    "Reassoc: No AP found or assoc failed." 
                    "Restarting re-assoc Timer @ %lu\n", os_time_get());
            
            
        
    
    
    
    



/** 
 *  @brief This function triggers re-association by waking up
 *  re-assoc thread.
 *  
 *  @param FunctionContext    A pointer to FunctionContext
 *  @return 	   n/a
 */ 
    void
wlan_reassoc_timer_func(void *FunctionContext) 
{
    
    
    
    
    
    
    
        
            /* wait until Exit_PS command returns */ 
            Adapter->ReassocTimerIsSet = TRUE;
        
        
                "for Exit_PS done\n", Adapter->PSState);
        
        
    
    
    
    
    



#endif  /* REASSOCIATION */