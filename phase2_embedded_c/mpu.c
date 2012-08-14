/*****************************************************************************
 * mpu.c
 * pquevedo 12/08/2012
 *
 * This driver provides the interface to control and configure the MPU.
 * The MPU consists of multiple region descriptors that can be configured.
 * For regions that overlap the attributes defined are logically OR'd together.
 * Region 0 is a unique region that is initialized by hardware to allow
 * the Core, Debugger and DMA full access to the entire memory space. The core
 * is only able to modify attributes (except the debuggers) for region 0.
 * It cannot modify start/end address or the enable/disable of that region
 *
 *****************************************************************************/
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

typedef struct {
    bool32_t initialized;
    uint32_t numRegions;
    uint32_t regionIds[MAX_MPU_REGIONS];
    void (*notifyFn[MAX_MPU_REGIONS])(const mpuFaultDesc_t *);
} mpuState_t;

/* Hardware initializes region 0 to allow the core, debugger and dma to access
 * all memory. It does not allow the core to modify start and end regions or
 * disable the debuggers access privillages. */
static mpuState_t mpuState = {
    .numRegions = 1,
    .regionIds[0] = MPU_REGION0_ID,
};

/*****************************************************************************
 * ISR Bus Fault Handler
 *
 *      Checks for faults on the MPU. If present the routine processes them,
 *  clears them and resumes execution. ISR hangs otherwise for debugging
 ****************************************************************************/
static void isrBusFaultHandler(void)
{
    if (!mpuCheckFaults()) {
        /* Not an MPU Error */
        while (1)
            ;
    }
}

/******************************************************************************
 * mpuInit
 *
 *      This routine installs an ISR handler for ISR_BUS_FAULT. Any access the
 *  core makes that violates an MPU region descriptor will cause a busFault
 *  exception to trip
 *****************************************************************************/
static inline void mpuInit(void)
{
    if (mpuState.initialized == FALSE) {
        mpuState.initialized  = TRUE;
        hwInstallISRHandler(ISR_BUS_FAULT, isrBusFaultHandler);
    }
}

/*****************************************************************************
 * Helper Functions
 ****************************************************************************/
static int32_t getRegionFromId(int32_t regionId)
{
    int region;
    for (region = 0; region < mpuState.numRegions; region++) {
        if (mpuState.regionIds[region] == regionId) {
            break;
        }
    }

    return region < mpuState.numRegions ? region : ERROR;
}

static mpuRGD_t *getRGD(int32_t region)
{
    switch (region) {
    default: assert(FALSE);
    case  0: return (mpuRGD_t *)MPU_RGD0_BASE_ADDR;
    case  1: return (mpuRGD_t *)MPU_RGD1_BASE_ADDR;
    case  2: return (mpuRGD_t *)MPU_RGD2_BASE_ADDR;
    case  3: return (mpuRGD_t *)MPU_RGD3_BASE_ADDR;
    case  4: return (mpuRGD_t *)MPU_RGD4_BASE_ADDR;
    case  5: return (mpuRGD_t *)MPU_RGD5_BASE_ADDR;
    case  6: return (mpuRGD_t *)MPU_RGD6_BASE_ADDR;
    case  7: return (mpuRGD_t *)MPU_RGD7_BASE_ADDR;
    case  8: return (mpuRGD_t *)MPU_RGD8_BASE_ADDR;
    case  9: return (mpuRGD_t *)MPU_RGD9_BASE_ADDR;
    case 10: return (mpuRGD_t *)MPU_RGD10_BASE_ADDR;
    case 11: return (mpuRGD_t *)MPU_RGD11_BASE_ADDR;
    }
}

static int32_t getSvAttrShift(int32_t busMaster)
{
    switch (busMaster) {
    default: assert(FALSE);
    case  0: return MPU_RGD_M0SM_SHIFT;
    case  1: return MPU_RGD_M1SM_SHIFT;
    case  2: return MPU_RGD_M2SM_SHIFT;
    case  3: return MPU_RGD_M3SM_SHIFT;
    }
}

static int32_t getUmAttrShift(int32_t busMaster)
{
    switch (busMaster) {
    default: assert(FALSE);
    case  0: return MPU_RGD_M0UM_SHIFT;
    case  1: return MPU_RGD_M1UM_SHIFT;
    case  2: return MPU_RGD_M2UM_SHIFT;
    case  3: return MPU_RGD_M3UM_SHIFT;
    case  4: return MPU_RGD_M4UM_SHIFT;
    case  5: return MPU_RGD_M5UM_SHIFT;
    case  6: return MPU_RGD_M6UM_SHIFT;
    case  7: return MPU_RGD_M7UM_SHIFT;
    }
}

/******************************************************************************
 * processFault
 *
 *      Determine the region at fault and call notifyFn with a description
 * of the fault
 *****************************************************************************/
static void processFault(uint32_t addr, uint32_t data)
{
    mpuFaultDesc_t faultDesc;
    int i;

    faultDesc.addr       = addr;
    faultDesc.master     = (data >> MPU_EDR_EMN_SHIFT)   & MPU_EDR_EMN_MASK;
    faultDesc.errorAttr  = (data >> MPU_EDR_EATTR_SHIFT) & MPU_EDR_EATTR_MASK;
    faultDesc.writeError = (data >> MPU_EDR_ERW_SHIFT)   & MPU_EDR_ERW_MASK;
    faultDesc.readError  = !faultDesc.writeError;

    /* Check which error regions tripped the fault */
    for (i = 0; i < MAX_MPU_REGIONS; i++) {
        if (data & (0x80000000 >> i)) {
            if (mpuState.notifyFn[i])
                mpuState.notifyFn[i](&faultDesc);
        }
    }
}

/******************************************************************************
 * mpuCheckFaults
 *
 *      This routine checks if there is an mpu fault then processes and clears
 *  faults. It should be called anytime one of the following ISRS is
 *  tripped and the MPU is enabled.
 *      ISR 32: ISR_DMA_ERROR
 *      ISR 94: ISR_ETHERNET_MAC_ERROR
 *      ISR 89: ISR_USB_OTG
 *      ISR 96: ISR_SDHC
 *
 * RETURNS: TRUE/FALSE if an MPU error is present
 *****************************************************************************/
#define MPU_SLAVE_PORT_ERRORS (MPU_SPERR0 | MPU_SPERR1 | MPU_SPERR2     \
                                          | MPU_SPERR3 | MPU_SPERR4)
bool32_t mpuCheckFaults(void)
{
    uint32_t faults = MPU_CESR;

    if (faults & MPU_SLAVE_PORT_ERRORS) {
        if (faults &  MPU_SPERR0) {
            processFault(MPU_EAR0, MPU_EDR0);
            MPU_CESR |= MPU_SPERR0;   /* Clear Error */
        }
        if (faults &  MPU_SPERR1) {
            processFault(MPU_EAR1, MPU_EDR1);
            MPU_CESR |= MPU_SPERR1;   /* Clear Error */
        }
        if (faults &  MPU_SPERR2) {
            processFault(MPU_EAR2, MPU_EDR2);
            MPU_CESR |= MPU_SPERR2;   /* Clear Error */
        }
        if (faults &  MPU_SPERR3) {
            processFault(MPU_EAR3, MPU_EDR3);
            MPU_CESR |= MPU_SPERR3;   /* Clear Error */
        }
        if (faults &  MPU_SPERR4) {
            processFault(MPU_EAR4, MPU_EDR4);
            MPU_CESR |= MPU_SPERR4;   /* Clear Error */
        }
    }

    return faults ? TRUE : FALSE;
}

/******************************************************************************
 * mpuModifyRegion
 *
 *      Modifies the region corresponding to the input ID. If modifying just
 * attributes then there is no issue with the region being temporarily disabled.
 * If the regions address is modified care should be taken that no foul
 * effects will be caused by a temporary delay in the region being invalid
 *
 * RETURNS: ERROR/OK
 *****************************************************************************/
int32_t mpuModifyRegion(int32_t regionId, const mpuRegion_t *regionCfg)
{
    int returnVal = ERROR;
    int region = getRegionFromId(regionId);

    assert(region < MAX_MPU_REGIONS);
    assert(regionCfg);

    mpuInit();

    if (region != ERROR) {
        volatile mpuRGD_t *mpuRGD = getRGD(region);
        uint32_t attr = 0;
        int i;

        /* Core cannot modify the start and end address of region 0 */
        if (region > 0) {
            if (regionCfg->startAddr < regionCfg->endAddr) {
                mpuRGD->word0 = (regionCfg->startAddr & MPU_RGD_ADDRESS_MASK);
                mpuRGD->word1 = (regionCfg->endAddr   & MPU_RGD_ADDRESS_MASK);
            }
            else {
                mpuRGD->word0 = (regionCfg->endAddr   & MPU_RGD_ADDRESS_MASK);
                mpuRGD->word1 = (regionCfg->startAddr & MPU_RGD_ADDRESS_MASK);
            }
        }

        for (i = 0; i < MAX_CROSSBAR_MASTERS; i++) {
            /* The first 4 masters have seperate rwx supervisor/user ctrls */
            if (i < 4) {
                /* Supervisor mode has same permissions as user */
                attr |= MPU_SV_ACCESS_USER << getSvAttrShift(i);

                if (regionCfg->attr[i] & MPU_ATTR_READ)
                    attr |= MPU_USR_ACCESS_R << getUmAttrShift(i);
                if (regionCfg->attr[i] & MPU_ATTR_WRITE)
                    attr |= MPU_USR_ACCESS_W << getUmAttrShift(i);
                if (regionCfg->attr[i] & MPU_ATTR_EXECUTE)
                    attr |= MPU_USR_ACCESS_X << getUmAttrShift(i);
            }
            /* The remaining bus masters only have one r/w ctrl */
            else {
                if (regionCfg->attr[i] & MPU_ATTR_WRITE)
                    attr |= MPU_USR_ACCESS_W << getUmAttrShift(i);
                if (regionCfg->attr[i] & MPU_ATTR_READ)
                    attr |= MPU_USR_ACCESS_R << getUmAttrShift(i);
            }
        }

        if (mpuRGD->word3 & MPU_RGD_VALID) {
            /* Use alternate access register so as to not negate valid bit */
            switch (region) {
            case  0: MPU_RGDAAC0  = attr; break;
            case  1: MPU_RGDAAC1  = attr; break;
            case  2: MPU_RGDAAC2  = attr; break;
            case  3: MPU_RGDAAC3  = attr; break;
            case  4: MPU_RGDAAC4  = attr; break;
            case  5: MPU_RGDAAC5  = attr; break;
            case  6: MPU_RGDAAC6  = attr; break;
            case  7: MPU_RGDAAC7  = attr; break;
            case  8: MPU_RGDAAC8  = attr; break;
            case  9: MPU_RGDAAC9  = attr; break;
            case 10: MPU_RGDAAC10 = attr; break;
            case 11: MPU_RGDAAC11 = attr; break;
            default: assert(FALSE); break;
            }
        }
        else {
            mpuRGD->word2 = attr;
        }

        if (regionCfg->enable)
            mpuRGD->word3 |=  MPU_RGD_VALID;
        else
            mpuRGD->word3 &= ~MPU_RGD_VALID;

        mpuState.notifyFn[region] = regionCfg->notifyFn;

        returnVal = OK;
    }

    return returnVal;
}

/*****************************************************************************
 * mpuAddRegion
 *
 *      Reserves and initializes a region descriptor. Returns a unique id
 *  used to allow exclusive access to modify the region configuration
 *
 *  RETURNS: regionId/ERROR
 ****************************************************************************/
int32_t mpuAddRegion(const mpuRegion_t *regionCfg)
{
    int returnVal = ERROR;

    assert(regionCfg);

    mpuInit();

    if (mpuState.numRegions < MAX_MPU_REGIONS) {
        int region = mpuState.numRegions++;

        mpuState.regionIds[region] = region + 255;

        if (mpuModifyRegion(mpuState.regionIds[region], regionCfg) != ERROR)
            returnVal = mpuState.regionIds[region];
    }

    return returnVal;
}

/******************************************************************************
 * mpuEnable
 *
 *      Enables/Disables the MPU. Hardware enables the MPU by default.
 *
 *  RETURNS: OK
 *****************************************************************************/
int32_t mpuEnable(bool32_t enable)
{
    mpuInit();

    if (enable)
        MPU_CESR |=  MPU_VLD;
    else
        MPU_CESR &= ~MPU_VLD;

    return OK;
}


