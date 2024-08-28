/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 series FMC driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

//* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "Mini57Series.h"
#include "fmc.h"

/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_FMC_Driver FMC Driver
  @{
*/


/** @addtogroup Mini57_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

int32_t  g_FMC_i32ErrCode;

/**
  * @brief    Disable all FMC functions
  *
  * @return   None
  */
void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief    Erase a page. The page size is 512 bytes.
  * @param[in]    u32PageAddr   Flash page address. Must be a 512-byte aligned address.
  * @return ISP page erase success or not.  
  * @retval   0   Success
  * @retval   -1   Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Erase failed or erase time-out  
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;
    	
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_ERASE;
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}


/**
  * @brief Execute FMC_ISPCMD_PAGE_ERASE command to erase SPROM. The page size is 512 bytes.
  * @param[in]    u32PageAddr   SPROM Flash page address. Must be a 512-byte aligned address.
  * @return   SPROM page erase success or not.
  * @retval   0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Erase failed or erase time-out   
  */
int32_t FMC_Erase_SPROM(uint32_t u32PageAddr)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;
    	
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPDAT = 0x0055AA03;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_ERASE;
    
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}


/**
  * @brief    get the current boot source
  * @retval   0   This chip is currently booting from APROM
  * @retval   1   This chip is currently booting from LDROM
  */
int32_t FMC_GetBootSource (void)
{
    if (FMC->ISPCTL & FMC_ISPCTL_BS_Msk)
        return 1;
    else
        return 0;
}


/**
  * @brief    Enable FMC ISP function
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief    Read a word from specified flash address.
  * @param[in]    u32Addr   Flash word address. Must be a word aligned address.
  * @return       The word data stored in the flash address "u32Addr".
  *           Return 0xFFFFFFFF if read failed.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out   
  */
uint32_t FMC_Read(uint32_t u32Addr)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;
    	
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_READ;
    
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }

    return FMC->ISPDAT;
}


/**
  * @brief    Read company ID
  * @param    None
  * @return   The company ID (32-bit). 0xFFFFFFFF means read failed.
  * @details  The company ID of Nuvoton is fixed to be 0xDA
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out 
  */
uint32_t FMC_ReadCID(void)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    FMC->ISPADDR = 0x0u;                         /* Must keep 0x0 when read CID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                           /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
        {
            if (FMC->ISPDAT != 0xDA)
                g_FMC_i32ErrCode = -1;
            return FMC->ISPDAT;
        }
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;

}


/**
  * @brief    Read product ID
  * @param    None
  * @return   The product ID (32-bit). 0xFFFFFFFF means read failed.
  * @details  This function is used to read product ID.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out 
  */
uint32_t FMC_ReadPID(void)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_PID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0x04u;                       /* Must keep 0x4 when read PID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                          /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}


/**
  * @brief    This function reads one of the four UCID.
  * @param[in]    u32Index   index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  * @return     The UCID of specified index. 0xFFFFFFFF means read failed.
  * @details    This function is used to read unique chip ID (UCID).
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out  
  */
uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
    FMC->ISPADDR = (0x04u * u32Index) + 0x10u;    /* The UCID is at offset 0x10 with word alignment. */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                            /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}


/**
  * @brief    This function reads one of the three UID.
  * @param[in]    u32Index  Index of the UID to read. u32Index must be 0, 1, or 2.
  * @return      The 32-bit unique ID data of specified UID index. 0xFFFFFFFF means read failed.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadUID(uint32_t u32Index)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;
    	
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = 0x04 * u32Index;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}


/**
  * @brief    Get the base address of Data Flash if enabled.
  * @return   The base address of Data Flash
  */
uint32_t FMC_ReadDataFlashBaseAddr(void)
{
    return FMC->DFBA;
}


/**
  * @brief    This function will force re-map assigned flash page to CPU address 0x0.
  * @param[in]    u32PageAddr   Address of the page to be mapped to CPU address 0x0.
  * @return      To set VECMAP to remap specified page address to 0x0.
  * @details     This function is used to set VECMAP to map specified page to vector page (0x0).
  * @retval      0   Success
  * @retval      -1  Failed
  * @note        Global error code g_FMC_i32ErrCode
  *              -1  Command time-out
  */
int32_t FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    uint32_t  tout = FMC_TIMEOUT_WRITE;

    g_FMC_i32ErrCode = 0;
    	
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif                                /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!FMC->ISPTRG)             /* Waiting for ISP Done */
            return 0;
    }
    g_FMC_i32ErrCode = -1;
    return -1;
}


/**
  * @brief    Obtain the current vector page address setting.
  * @return   The vector page address.
  */
uint32_t FMC_GetVectorPageAddr(void)
{
    return (FMC->ISPSTS & 0x0FFFFF00ul);
}



/**
  * @brief    Writes a word data to specified flash address.
  * @param[in]   u32Addr  Destination address
  * @param[in]   u32Data  Word data to be written
  * @return   0   Success
  * @return   -1  Program Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;
    	
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    
    tout = FMC_TIMEOUT_WRITE;
    
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;    
}


/**
  * @brief    Read the User Configuration words.
  * @param[in]    u32Config   The word array to store data.
  * @param[in]    u32Count    Maximum length of "u32Config".
  * @retval   0   Success
  * @retval   -1  Failed
  */
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count)
{
    if (u32Count < 1)
        return 0;

    u32Config[0] = FMC_Read(FMC_CONFIG_BASE);
    if (u32Count < 2)
        return 0;

    u32Config[1] = FMC_Read(FMC_CONFIG_BASE+4);
    return 0;
}


/**
  * @brief    Write User Configuration
  * @param[in]    u32Config  The word array to store data.
  * @param[in]    u32Count   Maximum length of "u32Config".
  * @retval   0   Success
  * @retval   -1  Failed
  */
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count)
{
    if (u32Count < 1)
        return 0;

    FMC_ENABLE_CFG_UPDATE();
    FMC_Erase(FMC_CONFIG_BASE);
    FMC_Write(FMC_CONFIG_BASE, u32Config[0]);

    if (u32Count < 2)
        return 0;

    FMC_Write(FMC_CONFIG_BASE+4, u32Config[1]);
    FMC_DISABLE_CFG_UPDATE();
    return 0;
}


/**
  * @brief    Calculate and read the CRC32 checksum of a specified flash area.
  * @param[in]    addr     Start address of the flash area to be executed CRC32 checksum calculation.
  * @param[in]    count    Number of bytes to be calculated.
  * @param[out]   chksum   If success, it will contain the result of CRC32 checksum calculation.
  * @retval   0   Success
  * @retval   -1  Invalid parameter or read checksum time-out failed.
  */
int32_t FMC_GetCRC32Sum(uint32_t addr, uint32_t count, uint32_t *chksum)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;
    	
    FMC->ISPCMD = FMC_ISPCMD_CAL_CRC32;
    FMC->ISPADDR = addr;
    FMC->ISPDAT  = count;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_CHKSUM;
    
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        return -1;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_CHKSUM;
    
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        return -1;
    }

    *chksum = FMC->ISPDAT;

    return 0;
}


/*@}*/ /* end of group Mini57_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_FMC_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/


