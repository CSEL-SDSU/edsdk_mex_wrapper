/* EDSDK MEX wrapper 
* Author: Jack Thomas Volponi 1/21/2026
* Description: This code was written to create a fast matlab interface with a Canon EOS 5D Mark III.
* It is used to sync recording and flowrate in the SDSU Narrow Channal Apparatus. It is very limited
* and only supports 1 camera with .CR2 and .MOV files. It is pretty fragile by itself. It is intended
* to be used insdie a matlab app which can control acess and such.
* Revision Log:
* 1/28/2026 JTV: Adding status function to return all state booleans as a MATLAB struct. Used to keep 
* track of the camera's state inside the MATLAB app.
* 2/02/2026 JTV: Adding cleanup on camera shutdown warning.
* 2/02/2026 later in day JTV: adding way to change apeture.
* 2/03/2026 JTV: adding way to change iso speed 
* 2/09/2026 JTV: Adding elapsed recording time 
* 3/13/2026 JTV: Adding way to download movies after they are made instead of immediately when the
* camera reports them. This is because movies take a long time to download and it is better to control
* when that happens from the matlab app so that the flow control is not locked while downloading. Also
* fixed bug that led to matlab memory access violation when the camera was shut off when matlab was also
* asking for frames.
* 6/08/2026 JTV: Added ability to adjust shutter speed. Added quering camera for possible settings instead of
* hard coding. 
*/

//Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <windows.h>
#include <string.h>
#include <stdbool.h>
#include <time.h> //Standard time library
#include <math.h>
#include <ctype.h>

//Include EDSDK headers
#include <EDSDK.h>
#include <EDSDKErrors.h>
#include <EDSDKTypes.h>

//Include jpeg headers
#include <turbojpeg.h>

//Include mex
#include "mex.h"
#include "matrix.h"

//static variables to manage state
static EdsCameraListRef gCameraList = NULL;
static EdsCameraRef gCamera = NULL;


//State Flags
static bool isSDKInitialized = false;
static bool isSessionOpen = false;
static bool eventHasFired = false;
static bool liveViewActive = false;
static bool frameSizeKnown = false; //Image size known flag
static bool recordingActive = false; 
static bool movieModeActive = false; 

// Image stuff
static EdsEvfImageRef gEvfImage = NULL;
static EdsStreamRef gEvfStream = NULL;
static uint8_t* frameBuffer = NULL;
static int width, height, subsamp, colorspace; 
static mxArray* mxImage = NULL;

// Jpeg decompressor
static tjhandle tj = NULL;

//Mex locked bool
static bool mexLocked = false;

// Live view stopped by download
static int notReadyCounter = 0;                 //counter for Camera not ready error
static volatile bool downloadingActive = false; //bool to flag active download, volatile to allow for multiple threads to access it.
// Volatile forces the variable to written and read to and from memoery every time it is referenced. It cannot be stored by caching it 
// on a cpu register

// Shutdown Event handling (Need to clear mex in main thread)
static volatile bool shutDownRequested = false; // Flag to queue a cleanup on shutdown
static volatile bool mexShutDownHandled = false; // Flag to indicate if the cleanup (called from shutdown has completed)

// Cache and que movie downloads 
static EdsDirectoryItemRef gPendingMovieItem = NULL; //Actual object that is created when event == kEdsObjectEvent_DirItemRequestTransfer || event == kEdsObjectEvent_DirItemCreated)
static EdsDirectoryItemInfo gPendingMovieInfo = { 0 };
static bool pendingMovieDownload = false; 

// Define Apeature Values for EOS 5D Mark III
typedef struct {
    EdsUInt32 eds_property_value;
    double apertureVal;
} apertureEntry;

// Array to store apeture values for the camera the CSEL owns (maybe populate with EdsGetPropDesc in future)
static const apertureEntry EOS5DMkIIIAv[] = {
  {0x20, 2.8},
  {0x23, 3.2},
  {0x24, 3.5},
  {0x28, 4.0},
  {0x2B, 4.5},
  {0x2D, 5.0},
  {0x30, 5.6},
  {0x33, 6.3},
  {0x35, 7.1},
  {0x38, 8.0},
  {0x3B, 9.0},
  {0x3D, 10.0},
  {0x40, 11.0},
  {0x44, 13.0},
  {0x45, 14.0},
  {0x48, 16.0},
  {0x4B, 18.0},
  {0x4D, 20.0},
  {0x50, 22.0},
  {0x53, 25.0},
  {0x55, 29.0},
  {0x58, 32.0},
};

// Save #of elements of apetures struct. 
// Divides the total number of bytes in the whole struct by the number of bytes only in one element
static const size_t EOS5DMkIIIAv_length = sizeof(EOS5DMkIIIAv) / sizeof(EOS5DMkIIIAv[0]); 

// Define ISO Speed values fo EOS 5D Mark III
typedef struct {
    EdsUInt32 eds_property_value;
    double isoSpeed_val;
} isoSpeedEntry;

// Structure Array to store valid iso values for camera CSEL owns (maybe populate with EdsGetPropDesc in future)
static const isoSpeedEntry EOS5DMkIII_iso[] = {
    {0x00000000, -1}, // Let AUTO correspond to -1
    {0x00000048, 100},
    {0x0000004b, 125},
    {0x0000004d, 160},
    {0x00000050, 200},
    {0x00000053, 250},
    {0x00000055, 320},
    {0x00000058, 400},
    {0x0000005b, 500},
    {0x0000005d, 640},
    {0x00000060, 800},
    {0x00000063, 1000},
    {0x00000065, 1250},
    {0x00000068, 1600},
    {0x0000006b, 2000},
    {0x0000006d, 2500},
    {0x00000070, 3200},
    {0x00000073, 4000},
    {0x00000075, 5000},
    {0x00000078, 6400},
    {0x0000007b, 8000},
    {0x0000007d, 10000},
    {0x00000080, 12800},
    {0x00000083, 16000},
    {0x00000085, 20000},
    {0x00000088, 25600},
};

// Define length of the table
static const size_t EOS5DMkIII_iso_length = sizeof(EOS5DMkIII_iso) / sizeof(EOS5DMkIII_iso[0]);

// Recording Time Variables (also see static bool recordingActive = false;)
static clock_t movStartClock;
static double movElapsedTime = 0.0;


/*------------------------Integer to value helper functions--------------------*/
#define EDS_VALUE_NOT_VALID   (-2.0) //Standard double to return if the conversion returns something not valid

/*------------------------------------------------------------------------------
* Function:   eds_iso_code_to_value
* Description:Maps the 32 bit integer code to an iso double from section
*             5.2.22 in the EDSDK API Programming Reference for the property
*             kEdsPropID_ISOSpeed.
* Parameters: EdsUInt32 iso_code - integer representign a possible aperture value
* Returns:    double - value for the ISO sensitvity.
* ----------------------------------------------------------------------------*/
#define EDS_VALUE_ISO_AUTO        (-1.0)
static double eds_iso_code_to_value(EdsUInt32 iso_code)
{
    switch (iso_code) {
    case 0x00000000: return EDS_VALUE_ISO_AUTO;       // ISO Auto

    case 0x00000028: return 6.0;
    case 0x00000030: return 12.0;
    case 0x00000038: return 25.0;
    case 0x00000040: return 50.0;
    case 0x00000048: return 100.0;
    case 0x0000004B: return 125.0;
    case 0x0000004D: return 160.0;
    case 0x00000050: return 200.0;
    case 0x00000053: return 250.0;
    case 0x00000055: return 320.0;
    case 0x00000058: return 400.0;
    case 0x0000005B: return 500.0;
    case 0x0000005D: return 640.0;
    case 0x00000060: return 800.0;
    case 0x00000063: return 1000.0;
    case 0x00000065: return 1250.0;
    case 0x00000068: return 1600.0;
    case 0x0000006B: return 2000.0;
    case 0x0000006D: return 2500.0;
    case 0x00000070: return 3200.0;
    case 0x00000073: return 4000.0;
    case 0x00000075: return 5000.0;
    case 0x00000078: return 6400.0;
    case 0x0000007B: return 8000.0;
    case 0x0000007D: return 10000.0;
    case 0x00000080: return 12800.0;
    case 0x00000083: return 16000.0;
    case 0x00000085: return 20000.0;
    case 0x00000088: return 25600.0;
    case 0x0000008B: return 32000.0;
    case 0x0000008D: return 40000.0;
    case 0x00000090: return 51200.0;
    case 0x00000093: return 64000.0;
    case 0x00000095: return 80000.0;
    case 0x00000098: return 102400.0;
    case 0x000000A0: return 204800.0;
    case 0x000000A8: return 409600.0;
    case 0x000000B0: return 819200.0;

    case 0xFFFFFFFFu: return EDS_VALUE_NOT_VALID;

    default: return NAN;
    }
}

/*------------------------------------------------------------------------------
* Function:   eds_av_code_to_fnumber
* Description:Maps the 32 bit integer code to a fnumber aperature double from section
*             5.2.25 in the EDSDK API Programming Reference for the property
*             kEdsPropID_Av.
* Parameters: EdsUInt32 av_code - integer representign a possible aperture value
* Returns:    double - value for the aperture.
* ----------------------------------------------------------------------------*/
static double eds_av_code_to_fnumber(EdsUInt32 av_code)
{
    switch (av_code) {
        case 0x00000008: return 1.0;
        case 0x0000000B: return 1.1;
        case 0x0000000C: return 1.2;
        case 0x0000000D: return 1.2;   // 1.2, 1/3-step variant
        case 0x00000010: return 1.4;
        case 0x00000013: return 1.6;
        case 0x00000014: return 1.8;
        case 0x00000015: return 1.8;   // 1.8, 1/3-step variant
        case 0x00000018: return 2.0;
        case 0x0000001B: return 2.2;
        case 0x0000001C: return 2.5;
        case 0x0000001D: return 2.5;   // 2.5, 1/3-step variant
        case 0x00000020: return 2.8;
        case 0x00000023: return 3.2;
        case 0x00000085: return 3.4;
        case 0x00000024: return 3.5;
        case 0x00000025: return 3.5;   // 3.5, 1/3-step variant
        case 0x00000028: return 4.0;
        case 0x0000002B: return 4.5;
        case 0x0000002C: return 4.5;
        case 0x0000002D: return 5.0;
        case 0x00000030: return 5.6;
        case 0x00000033: return 6.3;
        case 0x00000034: return 6.7;
        case 0x00000035: return 7.1;
        case 0x00000038: return 8.0;
        case 0x0000003B: return 9.0;
        case 0x0000003C: return 9.5;
        case 0x0000003D: return 10.0;
        case 0x00000040: return 11.0;
        case 0x00000043: return 13.0;  // 13, 1/3-step variant
        case 0x00000044: return 13.0;
        case 0x00000045: return 14.0;
        case 0x00000048: return 16.0;
        case 0x0000004B: return 18.0;
        case 0x0000004C: return 19.0;
        case 0x0000004D: return 20.0;
        case 0x00000050: return 22.0;
        case 0x00000053: return 25.0;
        case 0x00000054: return 27.0;
        case 0x00000055: return 29.0;
        case 0x00000058: return 32.0;
        case 0x0000005B: return 36.0;
        case 0x0000005C: return 38.0;
        case 0x0000005D: return 40.0;
        case 0x00000060: return 45.0;
        case 0x00000063: return 51.0;
        case 0x00000064: return 54.0;
        case 0x00000065: return 57.0;
        case 0x00000068: return 64.0;
        case 0x0000006B: return 72.0;
        case 0x0000006C: return 76.0;
        case 0x0000006D: return 80.0;
        case 0x00000070: return 91.0;

        case 0xFFFFFFFFu: return EDS_VALUE_NOT_VALID;

        default: return NAN;
    }
}

/*------------------------------------------------------------------------------
* Function:   eds_Tv_value_to_shutter_speed
* Description:Maps the 32 bit integer code to a shutter speed double from section
*             5.2.26 in the EDSDK API Programming Reference for the property
*             kEdsPropID_Tv.
* Parameters: EdsUInt32 tv_code - integer representign a possible shutter speed value
* Returns:    double - value for the shutter speed.
* ----------------------------------------------------------------------------*/
#define EDS_TV_BULB_SECONDS        (-1.0)
static double eds_Tv_value_to_shutter_speed(EdsUInt32 tv_code)
{
    switch (tv_code) {
    case 0x0000000C: return EDS_TV_BULB_SECONDS;      // Bulb

    case 0x00000010: return 30.0;                     // 30"
    case 0x00000013: return 25.0;                     // 25"
    case 0x00000014: return 20.0;                     // 20"
    case 0x00000015: return 20.0;                     // 20" 1/3-step display variant
    case 0x00000018: return 15.0;                     // 15"
    case 0x0000001B: return 13.0;                     // 13"
    case 0x0000001C: return 10.0;                     // 10"
    case 0x0000001D: return 10.0;                     // 10" 1/3-step display variant
    case 0x00000020: return 8.0;                      // 8"
    case 0x00000023: return 6.0;                      // 6" 1/3-step display variant
    case 0x00000024: return 6.0;                      // 6"
    case 0x00000025: return 5.0;                      // 5"
    case 0x00000028: return 4.0;                      // 4"
    case 0x0000002B: return 3.2;                      // 3"2
    case 0x0000002C: return 3.0;                      // 3"
    case 0x0000002D: return 2.5;                      // 2"5
    case 0x00000030: return 2.0;                      // 2"
    case 0x00000033: return 1.6;                      // 1"6
    case 0x00000034: return 1.5;                      // 1"5
    case 0x00000035: return 1.3;                      // 1"3
    case 0x00000038: return 1.0;                      // 1"
    case 0x0000003B: return 0.8;                      // 0"8
    case 0x0000003C: return 0.7;                      // 0"7
    case 0x0000003D: return 0.6;                      // 0"6
    case 0x00000040: return 0.5;                      // 0"5
    case 0x00000043: return 0.4;                      // 0"4
    case 0x00000044: return 0.3;                      // 0"3
    case 0x00000045: return 1.0 / 3.0;                // 0"3, exact 1/3

    case 0x00000048: return 1.0 / 4.0;                // 1/4
    case 0x0000004B: return 1.0 / 5.0;                // 1/5
    case 0x0000004C: return 1.0 / 6.0;                // 1/6
    case 0x0000004D: return 1.0 / 6.0;                // 1/6 1/3-step display variant
    case 0x00000050: return 1.0 / 8.0;                // 1/8
    case 0x00000053: return 1.0 / 10.0;               // 1/10 1/3-step display variant
    case 0x00000054: return 1.0 / 10.0;               // 1/10
    case 0x00000055: return 1.0 / 13.0;               // 1/13
    case 0x00000058: return 1.0 / 15.0;               // 1/15
    case 0x0000005B: return 1.0 / 20.0;               // 1/20 1/3-step display variant
    case 0x0000005C: return 1.0 / 20.0;               // 1/20
    case 0x0000005D: return 1.0 / 25.0;               // 1/25
    case 0x00000060: return 1.0 / 30.0;               // 1/30
    case 0x00000063: return 1.0 / 40.0;               // 1/40
    case 0x00000064: return 1.0 / 45.0;               // 1/45
    case 0x00000065: return 1.0 / 50.0;               // 1/50
    case 0x00000068: return 1.0 / 60.0;               // 1/60
    case 0x0000006B: return 1.0 / 80.0;               // 1/80
    case 0x0000006C: return 1.0 / 90.0;               // 1/90
    case 0x0000006D: return 1.0 / 100.0;              // 1/100
    case 0x00000070: return 1.0 / 125.0;              // 1/125
    case 0x00000073: return 1.0 / 160.0;              // 1/160
    case 0x00000074: return 1.0 / 180.0;              // 1/180
    case 0x00000075: return 1.0 / 200.0;              // 1/200
    case 0x00000078: return 1.0 / 250.0;              // 1/250
    case 0x0000007B: return 1.0 / 320.0;              // 1/320
    case 0x0000007C: return 1.0 / 350.0;              // 1/350
    case 0x0000007D: return 1.0 / 400.0;              // 1/400
    case 0x00000080: return 1.0 / 500.0;              // 1/500
    case 0x00000083: return 1.0 / 640.0;              // 1/640
    case 0x00000084: return 1.0 / 750.0;              // 1/750
    case 0x00000085: return 1.0 / 800.0;              // 1/800
    case 0x00000088: return 1.0 / 1000.0;             // 1/1000
    case 0x0000008B: return 1.0 / 1250.0;             // 1/1250
    case 0x0000008C: return 1.0 / 1500.0;             // 1/1500
    case 0x0000008D: return 1.0 / 1600.0;             // 1/1600
    case 0x00000090: return 1.0 / 2000.0;             // 1/2000
    case 0x00000093: return 1.0 / 2500.0;             // 1/2500
    case 0x00000094: return 1.0 / 3000.0;             // 1/3000
    case 0x00000095: return 1.0 / 3200.0;             // 1/3200
    case 0x00000098: return 1.0 / 4000.0;             // 1/4000
    case 0x0000009B: return 1.0 / 5000.0;             // 1/5000
    case 0x0000009C: return 1.0 / 6000.0;             // 1/6000
    case 0x0000009D: return 1.0 / 6400.0;             // 1/6400
    case 0x000000A0: return 1.0 / 8000.0;             // 1/8000
    case 0x000000A3: return 1.0 / 10000.0;            // 1/10000
    case 0x000000A5: return 1.0 / 12800.0;            // 1/12800
    case 0x000000A8: return 1.0 / 16000.0;            // 1/16000
    case 0x000000AB: return 1.0 / 20000.0;            // 1/20000
    case 0x000000AD: return 1.0 / 25600.0;            // 1/25600
    case 0x000000B0: return 1.0 / 32000.0;            // 1/32000

    case 0xFFFFFFFF: return EDS_VALUE_NOT_VALID; // Not valid / no settings change

    default: return NAN;                              // Unknown Tv code
    }
}

/*--------------------- END Integer to value helper functions--------------------*/

/*------------------------------------------------------------------------------
* Function:   build_unique_filename
* Description: Builds a unique filename by scanning existing .CR2 files in the current directory.
* Parameters: char* outFilename - Buffer to store the generated filename.
* 		 size_t n - Size of the output buffer.
* Returns:    char* - Pointer to the output filename buffer.
* ----------------------------------------------------------------------------*/
char* build_unique_filename(char* outFilename, size_t n, bool movieFlag)
{
    // Default Filenumber, number that is read from the file
    long val = 0; 
    long maxVal = 0;


    // Create pointers to the command null-terminated string which is stored in compiler managed memory
    const char* command = NULL;

	if (movieFlag) //movie branch
    {
        // Command to list all .MOV files in current directory
        command = "dir /B /A:-D *.MOV";
    }
    else //image branch
    {
		// Find all .CR2 files in current directory
        command = "dir /B /A:-D *.CR2";
    }
    

    char buf[256]; // buffer to hold each filename
	char filename[256]; // buffer to hold filename without extension

    // Open the terminal as a stream and read its output
    //FILE* fp = _popen(command);
    FILE* fp = _popen(command, "r");

    // Check if the stream opened successfully
    if (fp == NULL)
    {
        perror("popen failed");
        return NULL;
    }

    // Read each line from the stream until end of file
    while (fgets(buf, sizeof(buf), fp) != NULL)
    {
        // Print the output from the command
        printf("Filename: %s", buf);

        // Split the filename from the file extension      
		// Create a pointer to the last occurrence of '.' in the filename
		char* dot = strrchr(buf, '.'); 

		if (dot == NULL) // if no dot is found, do nothing
        {
			strcpy(filename, buf); // copy full filename
		}
        else
        {
			size_t name_len = dot - buf; // Calculate length of filename without extension
			strncpy(filename, buf, name_len); // Copy filename without extension
			filename[name_len] = '\0'; // Null-terminate the string
        }

        //Store the filename read from the stream (cmd)
        const char* currentFile = filename;

        //Extract the number from the file name
        char* p = currentFile; // Create pointer to first character in currentFile

        while (*p && !isdigit(*p)) //Loop over each character until a digit is found
        {
            p++; // Move pointer to the next character
        }

        // Read the number from characters at pointer location p to the next null character pointer
        // strtol will stop when it encounters a non-digit character, so it will not read the file extenison
        val = strtol(p, NULL, 10);
        if (val > maxVal)
        {
            maxVal = val;
        }

        //printf("Converted value: %0l4d \n", val);
    }

    // Close the stream
    _pclose(fp);

    //Safely format string to output filename 
    if (movieFlag)
    {
        snprintf(outFilename, n, "MOV_%04ld.MOV", maxVal + 1);
    }
    else
    {
        snprintf(outFilename, n, "IMG_%04ld.CR2", maxVal + 1);
    }   

    return outFilename;
}

/*------------------------------------------------------------------------------
* Function:   cleanup
* Description: Cleans up resources and handles errors.
* Parameters: EdsError err - Error code to handle.
* Returns:    None
* ---------------------------------------------------------------------------*/
void cleanup(EdsError err)
{
	//stop live view book keeping
    liveViewActive = false;
    frameSizeKnown = false;

    // Release live view resources 
    if (gEvfImage) { EdsRelease(gEvfImage); gEvfImage = NULL; }
    if (gEvfStream) { EdsRelease(gEvfStream); gEvfStream = NULL; }

    
    // Release decompressor
	if (tj) { tjDestroy(tj); tj = NULL; }

    // Release persistant matlab array
	if (mxImage) { mxDestroyArray(mxImage); mxImage = NULL; }

	// Close session and release camera
    if (isSessionOpen && gCamera) {
        EdsCloseSession(gCamera);
        isSessionOpen = false;
    }
    if (gCamera) { EdsRelease(gCamera); gCamera = NULL; }
    if (gCameraList) { EdsRelease(gCameraList); gCameraList = NULL; }

    if (isSDKInitialized) {
        EdsTerminateSDK();
        isSDKInitialized = false;
    }


    // Cleanup stored movie info
    if (gPendingMovieItem)
    {
        EdsRelease(gPendingMovieItem);
        gPendingMovieItem = NULL;
    }

    pendingMovieDownload = false; 
    memset(&gPendingMovieInfo, 0, sizeof(gPendingMovieInfo));

    // Cleanup movie mode
    movieModeActive = false;

    // Unlock Mex if it is locked
    if (mexLocked) {
        mexUnlock();
		mexLocked = false;
	}
	    
    /*
    if (cameraCSInitialized) {
        DeleteCriticalSection(&cameraCS);
        cameraCSInitialized = false;
    }*/


    // Handle error
    if (err != EDS_ERR_OK) {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "cmd failed. Error code: %d", (int)err);
    }
}

/*------------------------------------------------------------------------------
* Function:   download_directory_item
* Description: Helper that downloads a directory item (image or movie) reported
*              by the camera to a uniquely-named file on the host filesystem.
*              - Determines whether the incoming item is a movie (.MOV) or an
*                image (.CR2) by inspecting the directory item info.
*              - Builds a unique filename using `build_unique_filename`.
*              - Creates a host file stream and issues `EdsDownload`.
*              - Marks `downloadingActive` while the transfer is in progress
*                and calls `EdsDownloadComplete` when finished.
*              - Releases the created stream and returns the EDSDK error code.
* Parameters: EdsDirectoryItemRef object       - EDSDK directory item to download.
*             EdsDirectoryItemInfo* dirItemInfo - Pointer to item info (size, name).
* Returns:    EdsError - EDSDK status (EDS_ERR_OK on success).
* Notes:      - Caller is responsible for providing a valid `dirItemInfo`.
*             - This function updates the module global `downloadingActive`
*               flag to indicate active transfers.
* ---------------------------------------------------------------------------*/
static EdsError download_directory_item(EdsDirectoryItemRef object, const EdsDirectoryItemInfo* dirItemInfo)
{
    EdsError err = EDS_ERR_OK;
    EdsStreamRef stream = NULL;
    bool movieFlag = false; 
    EdsChar uniqueName[256];

    if (!object || !dirItemInfo) return EDS_ERR_INVALID_PARAMETER;

    //Create Unique Filename
    //determine if file is .mov or .cr2, image or video file
    //Camera will create ".MOV" file if it makes a movie
    movieFlag = strstr(dirItemInfo->szFileName, ".MOV") != NULL;

    if (!build_unique_filename(uniqueName, sizeof(uniqueName), movieFlag)) {
        err = EDS_ERR_INTERNAL_ERROR; // or another suitable code
    }
    if (err != EDS_ERR_OK) { printf("Error building filename. EdsError %d \n", err); }

    // Create File stream and download the image or video
    if (err == EDS_ERR_OK)
    {
        err = EdsCreateFileStream(uniqueName, kEdsFileCreateDisposition_CreateAlways, kEdsAccess_ReadWrite, &stream);
    }
    if (err != EDS_ERR_OK) { printf("Error creating filestream. EdsError %d \n", err); }

    if (err == EDS_ERR_OK)
    {
        downloadingActive = true;
        err = EdsDownload(object, dirItemInfo->size, stream);        
    }
    if (err != EDS_ERR_OK) { printf("Error downloading file. EdsError %d \n", err); }

    if (err == EDS_ERR_OK)
    {
        err = EdsDownloadComplete(object);
        if (err != EDS_ERR_OK)
        {
            printf("Error completeing download EdsError %d \n", err);
        }
                
    }
    else
    {
        EdsDownloadCancel(object);
    }

    downloadingActive = false;

    if (err == EDS_ERR_OK)
    {
        printf("Downloaded file: %s\n", uniqueName);
    }
    if (stream) 
    {
        EdsRelease(stream);
        stream = NULL;
    }

    return err; 

}

/*------------------------------------------------------------------------------
* Function:   handleObjectEvent
* Description: Callback function to handle object events from the camera.
* Parameters: EdsObjectEvent event - The ID of the event that occurs.
* 		   EdsBaseRef object - Reference to the object related to the event.
* 		   EdsVoid* context - User-defined context data (not used here).
* Returns:    EdsError - Error code indicating success or failure.
* ----------------------------------------------------------------------------*/
static EdsError EDSCALLBACK handleObjectEvent(EdsObjectEvent event, EdsBaseRef object, EdsVoid* context)
{
    EdsError err = EDS_ERR_OK;
    //printf("ObjectEvent: 0x%08X\n", event);
    if (event == kEdsObjectEvent_DirItemRequestTransfer || event == kEdsObjectEvent_DirItemCreated)
    {
        EdsDirectoryItemInfo dirItemInfo = { 0 };
		bool movieFlag = false;        

        // get file information, the file is the object for these events
        if (err == EDS_ERR_OK)
        {
            err = EdsGetDirectoryItemInfo(object, &dirItemInfo);
        }
        if (err != EDS_ERR_OK) { printf("Error getting directory item info. EdsError %d \n", err); }

        //Create Unique Filename
        //determine if file is .mov or .cr2, image or video file
        //Camera will create ".MOV" file if it makes a movie
        movieFlag = strstr(dirItemInfo.szFileName, ".MOV") != NULL;

        if (movieFlag)
        {
            // Hold onto movie for later download
            if (gPendingMovieItem) // Check for already story movie, release of already stored
            {
                EdsRelease(gPendingMovieItem);
                gPendingMovieItem = NULL;
            }

            // Store new movie file
            EdsRetain(object);
            gPendingMovieItem = (EdsDirectoryItemRef)object;
            gPendingMovieInfo = dirItemInfo;
            pendingMovieDownload = true;

            printf("Movie stored. Call edsdk_mex('downloadMovie') to download \n");
        }
        else //download images immediately.
        {
            err = download_directory_item((EdsDirectoryItemRef)object, &dirItemInfo);
            
            if (err != EDS_ERR_OK)
            {
                printf("photo download failed. EdsError %d\n", err);
            }

        }
        eventHasFired = true;

    }

    // Release objet when done
    if (object) EdsRelease(object);

    return err;
}

/*------------------------------------------------------------------------------
* Function:   handleStateEvent
* Description: Callback function to handle camera state events from the EDSDK.
*              Handles events such as shutdown, error and other camera state
*              transitions. Keep the callback minimal and thread-safe: do
*              best-effort EDSDK calls only, do NOT call any MATLAB API (mx* or mex*)
*              from this function.Signal the main / MATLAB thread(via a volatile
*              flag or OS event) for any MATLAB - thread cleanup.
*              Parameters: EdsStateEvent event - The ID of the state event that occurs.
*              EdsUInt32 parameter - Event - specific parameter(often unused).
*              EdsVoid * context - User - defined context pointer(may be NULL).
* Returns : EdsError - EDSDK error code indicating success(EDS_ERR_OK) or failure.
* ---------------------------------------------------------------------------- */
static EdsError EDSCALLBACK handleStateEvent(EdsStateEvent event, EdsUInt32 parameter, EdsVoid* context)
{
    EdsError err = EDS_ERR_OK;
    //printf("StateEvent: 0x%08X\n", event);

    if (event == kEdsStateEvent_Shutdown) //if camera is disconnected from computer
    {
        // Reset flags
        eventHasFired = true; 
        downloadingActive = false;
        liveViewActive = false; 
        recordingActive = false; 
        isSessionOpen = false;       
        movieModeActive = false;
        

        // Removing cleanup in call back. Cleanup is now deffered to the next edsdk call. Prevents asynchronus shutdown from separate threads.
        //Clear EDS related objects
        /*
        if (gEvfImage) { EdsRelease(gEvfImage); gEvfImage = NULL; }
        if (gEvfStream) { EdsRelease(gEvfStream); gEvfStream = NULL; }
        if (gCamera) { EdsRelease(gCamera); gCamera = NULL; }
        if (gCameraList) { EdsRelease(gCameraList); gCameraList = NULL; }

        //Clear turbo jpeg stuff 
        if (tj) { tjDestroy(tj); tj = NULL; }
                           
                           */
        // Reset frame related counters and flags
        frameSizeKnown = false;
        notReadyCounter = 0;

        // Request shutdown in main thread. Matlab mex objects assume they are 
        // on the main matlab managed thread.
        shutDownRequested = true; 

        printf("Camera was disconnected from PC. shutDownRequested = %d\n", shutDownRequested);
        printf("Please call edsdk_mex('getState') to unlock mex from the main thread. \n");
    }

    return err;
}

/*------------------------------------------------------------------------------
* Function:    cmd_downloadMovie
* Description: Function to download a stored movie directory item, gPendingMovieItem.
*              The function checks if there is a stored item and if there is one,
*              it downloads the item to the host pc and resets the movie info varible
*              to zeros. 
* Returns : EDSDK Errors
* ---------------------------------------------------------------------------- */
void cmd_downloadMovie(void)
{
    EdsError err = EDS_ERR_OK;

    if (!pendingMovieDownload || !gPendingMovieItem)
    {
        printf("no stored movie \n");
        return;
    }

    err = download_directory_item(gPendingMovieItem, &gPendingMovieInfo);
    if (err != EDS_ERR_OK)
    {
        printf("movie download failed. EdsError %d \n",err);
        return;
    }

    // Release directory item now that were done with it
    EdsRelease(gPendingMovieItem);

    // Reset flags and stored information
    gPendingMovieItem = NULL;

    // Fill memoery location of movie info with zeros that takes up the same amount of memory as gPendingMovieInfo
    memset(&gPendingMovieInfo, 0, sizeof(gPendingMovieInfo));
    pendingMovieDownload = false; 
}

/*------------------------------------------------------------------------------
* Function:   cmd_init
* Description: Initializes the EDSDK and connects to the first available camera.
* Parameters: None
* Returns:    EDSDK Errors
* ---------------------------------------------------------------------------*/
void cmd_init(void)
{
    if (isSDKInitialized) 
    {
		return; // Already initialized
	}

    // Define Variables
    EdsError err = EDS_ERR_OK;
    //EdsCameraListRef cameraList = NULL;
    //EdsCameraRef camera = NULL;
    EdsUInt32 camCount = 0;

    // Initialize EDSDK
    err = EdsInitializeSDK();
    if (err == EDS_ERR_OK)
    {
        isSDKInitialized = true;
    }
    else
    {
        cleanup(err);
	}

    // Find the Camera and Connect to it
    if (err == EDS_ERR_OK)
    {
        err = EdsGetCameraList(&gCameraList);
    }
    else
    {
        cleanup(err);
    }

    if (err == EDS_ERR_OK)
    {
        err = EdsGetChildCount(gCameraList, &camCount);
    }
    else
    {
        cleanup(err);
    }

    printf("Number of cameras: %u\n", camCount);
    if (err == EDS_ERR_OK)
    {
        err = EdsGetChildAtIndex(gCameraList, 0, &gCamera);
    }
    else
    {
        cleanup(err);
    }

    if (err == EDS_ERR_OK)
    {
        printf("Camera connected.\n");
    }
    else
    {
        cleanup(err);
    }

    // Set Event Handler
    // Object Event
    if (err == EDS_ERR_OK)
    {
        err = EdsSetObjectEventHandler(gCamera, kEdsObjectEvent_All, handleObjectEvent, NULL);
        printf("set object event handler err = %u\n", err);
    }
    else
    {
        cleanup(err);
    }

    // State Events
    if (err == EDS_ERR_OK)
    {
        err = EdsSetCameraStateEventHandler(gCamera, kEdsStateEvent_All, handleStateEvent, NULL);
    }
    else
    {
        cleanup(err);
    }

    // Enable Movie switch properties
    EdsUInt32 id;
    id = kEdsPropID_FixedMovie;
    
    if (err == EDS_ERR_OK)
    {
        err = EdsSetPropertyData(gCamera, 0x01000000, 0x17AF25B1, sizeof(id), &id);
    }
    
    // Open Session
    if (err == EDS_ERR_OK)
    {
        err = EdsOpenSession(gCamera);
        isSessionOpen = (err == EDS_ERR_OK);
        printf("open session err = %u\n", err);
    }
    else
    {
        cleanup(err);
    }

    // Tell Camera to save to Host PC 
    EdsUInt32 saveTo = kEdsSaveTo_Host;

    if (err == EDS_ERR_OK)
    {
        err = EdsSetPropertyData(gCamera, kEdsPropID_SaveTo, 0, sizeof(saveTo), &saveTo);
    }
    else
    {
        cleanup(err);
    }

    // Allocate Disk Space on Host PC for Camera to write to 
    EdsCapacity cameraCapacity;
    //Alocate 2GB of space
    cameraCapacity.numberOfFreeClusters = 0x7FFFFFFF;
    cameraCapacity.bytesPerSector = 512;
    cameraCapacity.reset = 1;

    if (err == EDS_ERR_OK)
    {
        err = EdsSetCapacity(gCamera, cameraCapacity);
    }
    else
    {
		cleanup(err);
    }

    if (err != EDS_ERR_OK) { printf("error allocating disk space for camera, Eds error: %d", err); }

    // initialize critical session (make code serial)
    /*
    if (err == EDS_ERR_OK)
    {
        if (!cameraCSInitialized) {
            InitializeCriticalSection(&cameraCS);
            cameraCSInitialized = true;
        }
    }
    else
    {
        printf("error allocating disk space for camera, Eds error: %d", err);
    }*/

	return;
}

/*------------------------------------------------------------------------------
* Function:   cmd_terminate
* Description: Terminates the EDSDK and releases resources.
* Parameters: None
* Returns:    EDSDK Errors
* --------------------------------------------------------------------------*/
void cmd_terminate(void)
{
    if (!isSDKInitialized)
    {
        return; // Not initialized
    }

    EdsError err = EDS_ERR_OK;

    if (isSessionOpen && gCamera) {
        EdsCloseSession(gCamera);
        isSessionOpen = false;
    }
    if (gCamera) {
        err = EdsRelease(gCamera);
        gCamera = NULL;
    }
    if (gCameraList) {
        err = EdsRelease(gCameraList);
        gCameraList = NULL;
	}
    if (isSDKInitialized) {
		EdsTerminateSDK();
		isSDKInitialized = false;
    }
    if (gEvfImage) {
        EdsRelease(gEvfImage);
        gEvfImage = NULL;
	}
    if (gEvfStream) {
        EdsRelease(gEvfStream);
        gEvfStream = NULL;
    }
    if (tj) {
        tjDestroy(tj);
        tj = NULL;
	}
    if (err != EDS_ERR_OK) {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "cmd_terminate failed. Error code: %d", (int)err);
    }    

    // Cleanup stored movie info
    if (gPendingMovieItem)
    {
        EdsRelease(gPendingMovieItem);
        gPendingMovieItem = NULL;
    }

    pendingMovieDownload = false;
    memset(&gPendingMovieInfo, 0, sizeof(gPendingMovieInfo));

    movieModeActive = false; 

    if (mexLocked) {
        mexUnlock();
		mexLocked = false;
    }

    /*
    if (cameraCSInitialized) {
        DeleteCriticalSection(&cameraCS);
        cameraCSInitialized = false;
    }*/

    return;
}

/*------------------------------------------------------------------------------
* Function:   cmd_takePhoto
* Description: Commands the camera to take a photo and waits for the event.
* Parameters: None
* Returns:    EDSDK Errors
* --------------------------------------------------------------------------*/
void cmd_takePhoto(void)
{
    EdsError err = EDS_ERR_OK;

    // Take picture
    if (err == EDS_ERR_OK)
    {
        err = EdsSendCommand(gCamera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_Completely);
    }
    else
    {
        cleanup(err);
	}

    if (err == EDS_ERR_OK)
    {
        err = EdsSendCommand(gCamera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_OFF);
    }
    else
    {
		cleanup(err);
    }

    // Capture Event
    const int maxWaitIterations = 1000;
    int waitIterations = 0;
    while (!eventHasFired && waitIterations < maxWaitIterations)
    {
        if (err == EDS_ERR_OK)
        {
            // Process Events
            err = EdsGetEvent();
            
        }
        else
        {
			cleanup(err);
        }
        Sleep(2); // Sleep for 1 ms to avoid busy loop

        waitIterations++;
		printf("Waiting for event... %d\n", waitIterations);
    }

	return;

}

/*------------------------------------------------------------------------------
* Function:   cmd_startLiveView
* Description: Starts the live view mode on the camera and prepares to receive live view images.
* Parameters: None
* Returns:    None
* --------------------------------------------------------------------------*/
void cmd_startLiveView(void)
{
    if (!isSDKInitialized || !isSessionOpen)
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "SDK not initialized or session not open.");
    }
    if (liveViewActive)
    {
        return; // Live view already active
    }

    EdsError err = EDS_ERR_OK;
       
    // Get the current output device property for the live view image
    EdsUInt32 outputDevice;
    err = EdsGetPropertyData(gCamera, kEdsPropID_Evf_OutputDevice, 0, sizeof(outputDevice), &outputDevice);

    // Start PC live view by setting PC as the output device for live view image
    if (err == EDS_ERR_OK)
    {
        // Only set the PC bit to start live view on PC 
        // This statement only changes output device bits to include PC without altering other bits which represent other output devices
        outputDevice |= kEdsEvfOutputDevice_PC;

        err = EdsSetPropertyData(gCamera, kEdsPropID_Evf_OutputDevice, 0, sizeof(outputDevice), &outputDevice);
    }
    else
    {
        cleanup(err);
    }

    // Create EVF Stream
    if (err == EDS_ERR_OK)
    {
        err = EdsCreateMemoryStream(0, &gEvfStream);
    }
    else
    {
        cleanup(err);
    }

    // Create EVF Image Ref. Creates an object used to get the live view image data set.
    if (err == EDS_ERR_OK)
    {
        err = EdsCreateEvfImageRef(gEvfStream, &gEvfImage);
    }
    else
    {
        cleanup(err);
    }

	//Set live view flag to on and set frame size known to false
    if (err == EDS_ERR_OK)
    {
        liveViewActive = true;
		frameSizeKnown = false;
    }
    else
    {
        cleanup(err);
	}

    // Set mexLock() to prevent mex file from being cleared while live view is active
    if (!mexLocked) {
        mexLock();
        mexLocked = true;
    }

    return;
}

/*------------------------------------------------------------------------------
* Function:   stopLiveView
* Description: Stops the live view mode on the camera.
* Parameters: None
* Returns:    None
* --------------------------------------------------------------------------*/
void cmd_stopLiveView(void)
{
    //if (!liveViewActive || !gCamera || !gEvfImage || !gEvfStream) {
    //    mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
    //        "Live view not properly initialized (camera/stream/image invalid).");
    //}
       
    EdsError err = EDS_ERR_OK;

    // Check for recording
    if (recordingActive)
    {
        printf("end movie recording before stopping live view. Issue command edsdk_mex(''stopMovie'')");
        return;
    }
        
    if (gCamera)
    {
		// turn off output device to stop live view
        // Get the output device property for the live view image
        EdsUInt32 outputDevice;

        // Set the output device variable to the current camera settings
        err = EdsGetPropertyData(gCamera, kEdsPropID_Evf_OutputDevice, 0, sizeof(outputDevice), &outputDevice);

        // End PC live view by removing the PC output device bit
        if (err == EDS_ERR_OK)
        {
            outputDevice &= ~kEdsEvfOutputDevice_PC; // Clear the PC bit
            err = EdsSetPropertyData(gCamera, kEdsPropID_Evf_OutputDevice, 0, sizeof(outputDevice), &outputDevice);

            if (err != EDS_ERR_OK) { printf("Error stopping live view EdsError: %d\n", err); }

        }
        
    }

    liveViewActive = false; 
	frameSizeKnown = false;

	// Release EVF Image Ref
    if (gEvfImage)
    {
        EdsRelease(gEvfImage);
        gEvfImage = NULL;
	}
    // Release EVF Stream
    if (gEvfStream)
    {
        EdsRelease(gEvfStream);
        gEvfStream = NULL;
	}

    // Release TurboJPEG decompressor
    if (tj)
    {
        tjDestroy(tj);
        tj = NULL;
	}

    // Release matlab array
    if (mxImage) { mxDestroyArray(mxImage); mxImage = NULL; }

    // Release mex lock
    if (mexLocked) {
        mexUnlock();
		mexLocked = false;
    }   

    return;
}

/*------------------------------------------------------------------------------
* Function:   resetEvf
* Description: Releases and recreates the EVF memory stream and EVF image
*              reference used for live-view. Resets internal state so the next
*              successful frame forces a header read (frame size discovery).
* Parameters: None
* Returns:    None
* Notes:      - Safe to call when live view is active to recover from repeated
*                download failures or device busy conditions.
*             - On error the function returns early (no MATLAB API calls).
*             - Caller should ensure any required synchronization (critical
*               sections) is held if called from multi-threaded contexts.
* --------------------------------------------------------------------------*/
static void resetEvf(void)
{
    EdsError err = EDS_ERR_OK;

    // Get the current output device property for the live view image
    EdsUInt32 outputDevice;
    err = EdsGetPropertyData(gCamera, kEdsPropID_Evf_OutputDevice, 0, sizeof(outputDevice), &outputDevice);

    // Start PC live view by setting PC as the output device for live view image
    if (err == EDS_ERR_OK)
    {
        // Only set the PC bit to start live view on PC 
        // This statement only changes output device bits to include PC without altering other bits which represent other output devices
        outputDevice |= kEdsEvfOutputDevice_PC;

        err = EdsSetPropertyData(gCamera, kEdsPropID_Evf_OutputDevice, 0, sizeof(outputDevice), &outputDevice);
    }
    else
    {
        cleanup(err);
    }

    // release evf objects 
    if (gEvfImage) { EdsRelease(gEvfImage);  gEvfImage = NULL; }
    if (gEvfStream) { EdsRelease(gEvfStream); gEvfStream = NULL; }

    // Remake evf objects
    err = EdsCreateMemoryStream(0, &gEvfStream);
    if (err != EDS_ERR_OK) return;

    err = EdsCreateEvfImageRef(gEvfStream, &gEvfImage);
    if (err != EDS_ERR_OK) return;

    frameSizeKnown = false;
}

/*------------------------------------------------------------------------------
* Function:   cmd_getFrame
* Description: Downloads the current live-view EVF frame from the camera, decodes
*              the embedded JPEG into an RGB image and returns a MATLAB uint8
*              array with dimensions (height x width x 3).
* Parameters: None
* Returns:    mxArray* - Pointer to a MATLAB array (mxUINT8_CLASS) containing the
*                       decoded image. On failure the function may return an
*                       empty mxArray (0x0) or raise a MATLAB error via
*                       mexErrMsgIdAndTxt for fatal conditions.
* Notes:      - Reuses the last decoded frame when the camera is busy.
*             - Requires live view to be active and EVF structures to be valid.
*             - Caller (gateway) typically calls mxDuplicateArray on the
*               returned pointer to produce a MATLAB-owned copy.
* --------------------------------------------------------------------------*/
mxArray* cmd_getFrame(void)
{
    if (!liveViewActive)
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "Live view not active.");
	}

	EdsError err = EDS_ERR_OK;
    bool downloaded = false;
    const int maxNotReadyCounter = 30; // Maximum # of getFrame fails before restarting evf stream

    // Enter Serial session
    //EnterCriticalSection(&cameraCS);

    // Check for invalid objects
    if (!gCamera || !gEvfImage || !gEvfStream) {
        //LeaveCriticalSection(&cameraCS);
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError", "EVF structures invalid in getFrame.");
    }

    // Download the first frame, store it in global EvfImage 
    err = EdsDownloadEvfImage(gCamera, gEvfImage);
    

    if (err == EDS_ERR_OK) //download successful
    {
        downloaded = true;
        notReadyCounter = 0;
    }
    else if (err == EDS_ERR_OBJECT_NOTREADY || err == EDS_ERR_DEVICE_BUSY) // camera busy, reuse stored frame
    {
        downloaded = false;
        

        if (!downloadingActive) // check if downloading is active
        {
            notReadyCounter++; // count number of not ready errors

            if (notReadyCounter >= maxNotReadyCounter) // if not ready for a while, evf probably needs to be reset
            {
                resetEvf();
                notReadyCounter = 0;
            }
        }
    }
    else // Other error
    {
        //LeaveCriticalSection(&cameraCS);
		printf("EdsDownloadEvfImage error: %d\n", err);
		return mxCreateDoubleMatrix(0, 0, mxREAL); // return empty array on error, do not call
    }

    
    /*
    if (notReadyCounter > maxnotReadyCounter) // if frame fails exceeds max fails, restart evf stream
    {
        resetEvf();
    }*/

    //LeaveCriticalSection(&cameraCS);

    // if downloaded is false, return the stored frame
    if (!downloaded)
    {
        if (mxImage) return mxImage;
		return mxCreateDoubleMatrix(0, 0, mxREAL); // return empty array if no stored frame
    }

    // Decode the image outside of single thread
    // Get the number of jpeg bytes and a pointer to where they start
    EdsUInt64 jpegSize = 0;
    EdsVoid* jpegPtr = NULL;

    if (err == EDS_ERR_OK)
    {
        err = EdsGetLength(gEvfStream, &jpegSize);
    }
    else
    {
        cleanup(err);
    }

    if (err == EDS_ERR_OK)
    {
        err = EdsGetPointer(gEvfStream, &jpegPtr);
    }
    else
    {
        cleanup(err);
    }

    if (err != EDS_ERR_OK) cleanup(err);

	// Initialize TurboJPEG decoder if not already done
    if (tj == NULL)
    {
        // Create TurboJPEG decoder object
        tj = tjInitDecompress();
        if (tj == NULL)
        {
            mexErrMsgIdAndTxt("eds:turbojpeg", "tjInitDecompress failed");
        }
    }

    // Convert the pointer to the first byte of the jpeg data to a raw byte pointer form the EDSDK void pointer 
    unsigned char* jpegBytes = (unsigned char*)jpegPtr;


	// Branch if the frame size is not known, calculate it
    if (!frameSizeKnown)
    {        
        // Decompress the JPEG header to get image dimensions and format
        tjDecompressHeader3(tj, jpegBytes, jpegSize, &width, &height, &subsamp, &colorspace);

        
        int pitch = width * 3; // RGB has 3 bytes per pixel
        int rgbSize = pitch * height;

        // Allocate memory for decodede RGB image
        unsigned char* rgbImage = (unsigned char*)mxMalloc(rgbSize);

		// Decompress the JPEG image to RGB format
        // TJPF_RGB tells the function to do an RBG output
		// TJFLAG_FASTDCT tells the function to use fast DCT algorithm for speed over quality
		// decompressed image is stored in rgbImage buffer
        // the RGB triplets are stored in a 1D array row by row
		tjDecompress2(tj, jpegBytes, jpegSize, rgbImage, width, pitch, height, TJPF_RGB, TJFLAG_FASTDCT);

        // Height, Width, Channels. Array of dimensions for mxArray
		mwSize dims[3] = { height, width, 3 }; 

        //Create a 3D mxArray for the RGB image
		mxImage = mxCreateNumericArray(3, dims, mxUINT8_CLASS, mxREAL);
		mexMakeArrayPersistent(mxImage); // Make mxArray persistent to avoid being cleared by matlab memory management

        // Set frame size known flag to true
        frameSizeKnown = true;

		// Get pointer to the data in the mxArray.
        // mxArray is owned by matlab. This accesses matlabs memory
		unsigned char* mxDataPtr = (unsigned char*)mxGetUint8s(mxImage);

		// Copy and Reorder the array from RGB to MATLAB's expected format which is R,G,B channels in 3rd dimension
        // Matlab/ fortran will read the 1d real memory array as all the rows of the first column, all the rows of the second
		// column, etc. So we need to rearrange the data from RGBRGBRGB... to RRR...GGG...BBB...
		for (int y = 0; y < height; y++) // Loop over rows
        {
			for (int x = 0; x < width; x++) // Loop over columns
            {
				int rbgIdx = (y * width + x) * 3; // Index in the RGB image (1D array index), pixel location index

				int mxIdxR = y + x * height; // Index for Red channel in mxArray
				int mxIdxG = y + x * height + height * width; // Index for Green channel
				int mxIdxB = y + x * height + 2 * height * width; // Index for Blue channel

				// Copy and reorder
				mxDataPtr[mxIdxR] = rgbImage[rbgIdx];     // Red
				mxDataPtr[mxIdxG] = rgbImage[rbgIdx + 1]; // Green
				mxDataPtr[mxIdxB] = rgbImage[rbgIdx + 2]; // Blue
            }
        }

		// Free the temporary RGB image buffer that is in C format 
        mxFree(rgbImage);		

        return mxImage;
    }
	else // Frame size is known, just decode directly into existing mxArray
    {
		// use previously known width, height to decode directly into matlab mxArray
        int pitch = width * 3; // RGB has 3 bytes per pixel
        int rgbSize = pitch * height;

        // Allocate memory for decodede RGB image
        unsigned char* rgbImage = (unsigned char*)mxMalloc(rgbSize);

        // Decompress the JPEG image to RGB format
        int rc = tjDecompress2(tj, jpegBytes, jpegSize, rgbImage, width, pitch, height, TJPF_RGB, TJFLAG_FASTDCT);

        if (rc != 0)
        {
			frameSizeKnown = false; // Reset frame size known flag on error

            return mxCreateDoubleMatrix(0, 0, mxREAL); // return []
		}

        // Get pointer to the data in the mxArray.
        // mxArray is owned by matlab. This accesses matlabs memory
        unsigned char* mxDataPtr = (unsigned char*)mxGetUint8s(mxImage);

        // Copy and Reorder the array from RGB to MATLAB's expected format which is R,G,B channels in 3rd dimension
        // Matlab/ fortran will read the 1d real memory array as all the rows of the first column, all the rows of the second
        // column, etc. So we need to rearrange the data from RGBRGBRGB... to RRR...GGG...BBB...
        for (int y = 0; y < height; y++) // Loop over rows
        {
            for (int x = 0; x < width; x++) // Loop over columns
            {
                int rbgIdx = (y * width + x) * 3; // Index in the RGB image (1D array index), pixel location index

                int mxIdxR = y + x * height; // Index for Red channel in mxArray
                int mxIdxG = y + x * height + height * width; // Index for Green channel
                int mxIdxB = y + x * height + 2 * height * width; // Index for Blue channel

                // Copy and reorder
                mxDataPtr[mxIdxR] = rgbImage[rbgIdx];     // Red
                mxDataPtr[mxIdxG] = rgbImage[rbgIdx + 1]; // Green
                mxDataPtr[mxIdxB] = rgbImage[rbgIdx + 2]; // Blue
            }
        }

		// Free the temporary RGB image buffer that is in C format
		mxFree(rgbImage);

		return mxImage;
    } 

}


void cmd_setMovieMode(bool movieModeOn)
{
    // allow if initialized and open and camera exists and not currently recording
    if (!isSDKInitialized || !isSessionOpen || gCamera == NULL)
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "SDK not initialized or session not open.");
    }

    if (recordingActive)
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:MovieModeError",
            "Cannot change movie mode while recording.");
    }

    
    EdsError err = EDS_ERR_OK;

    // Check Movie mode
    // movieMode 0 : Disable , 1 : Enable
    EdsUInt32 movieMode = 0;
    if (err == EDS_ERR_OK)
    {
        err = EdsGetPropertyData(gCamera, kEdsPropID_FixedMovie, 0, sizeof(movieMode), &movieMode);
    }
    if (err != EDS_ERR_OK)
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "Error getting movie mode state. EdsError: %d", (int)err);
    }

    movieModeActive = (movieMode != 0);

    if (movieModeActive == movieModeOn)
    {
        return;
    }

    // Set movie mode if it needs to be changed
    if (movieModeOn)
    {
        EdsUInt32 saveTo = kEdsSaveTo_Camera;
        err = EdsSetPropertyData(gCamera, kEdsPropID_SaveTo, 0, sizeof(saveTo), &saveTo);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "Error setting SaveTo to camera. EdsError: %d", (int)err);
        }

        err = EdsSendCommand(gCamera, kEdsCameraCommand_MovieSelectSwON, 0);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "Error turning on movie mode. EdsError: %d", (int)err);
        }

        movieModeActive = true;
    }
    else
    {
        //turn off movie mode and set the save location back to both
        err = EdsSendCommand(gCamera, kEdsCameraCommand_MovieSelectSwOFF, 0);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "Error turning off movie mode. EdsError: %d", (int)err);
        }
        movieModeActive = false;

        EdsUInt32 saveTo = kEdsSaveTo_Host;
        err = EdsSetPropertyData(gCamera, kEdsPropID_SaveTo, 0, sizeof(saveTo), &saveTo);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "Error setting SaveTo to host. EdsError: %d", (int)err);
        }

        EdsCapacity cameraCapacity;
        cameraCapacity.numberOfFreeClusters = 0x7FFFFFFF;
        cameraCapacity.bytesPerSector = 512;
        cameraCapacity.reset = 1;

        err = EdsSetCapacity(gCamera, cameraCapacity);

        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "Error setting host capacity. EdsError: %d", (int)err);
        }
    }    

    // Check host capacity after switchign back to host save

}
/*------------------------------------------------------------------------------
* Function:   cmd_startMovie
* Description: Starts movie recording on the camera. Switches the save location
*              to the camera (required for movie recording), enables movie mode
*              if necessary, and issues the record-start property.
* Parameters: None
* Returns:    None
* Notes:      - Requires live view to be active.
*             - Errors are reported via printf; fatal conditions are not raised
*               here so caller may continue running.
* --------------------------------------------------------------------------*/
void cmd_startMovie(void)
{
    if (!liveViewActive)
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "Live view not active.");
    }
    if (recordingActive)
    {
        return;
    }

	EdsError err = EDS_ERR_OK;

    //set save to location to Camera. Cannot directly save movie to PC
    /*EdsUInt32 saveTo = kEdsSaveTo_Camera;
    err = EdsSetPropertyData(gCamera, kEdsPropID_SaveTo, 0, sizeof(saveTo), &saveTo);
    if (err != EDS_ERR_OK) { printf("Error seting save to location to camera. EdsError: %d\n", err); }*/
    
    // Check Movie mode
    EdsUInt32 movieMode;
    if (err == EDS_ERR_OK) 
    {
        err = EdsGetPropertyData(gCamera, kEdsPropID_FixedMovie, 0, sizeof(movieMode), &movieMode);
    }
    if (err != EDS_ERR_OK) { printf("Error getting movie mode state. EdsError: %d\n", err); }

    if (movieMode != 1)
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "movie mode not active.");
        return; 
    }

    // Set movie mode to on
    /*if (err == EDS_ERR_OK)
    {
        if (movieMode == 0)
        {
            err = EdsSendCommand(gCamera, kEdsCameraCommand_MovieSelectSwON, 0);
        }
    }
    if (err != EDS_ERR_OK) { printf("Error turning on movie mode. EdsError: %d\n", err); }  */  
    
    // Begin movie shooting
    EdsUInt32 record_start = 4;
    if (err == EDS_ERR_OK)
    {
        err = EdsSetPropertyData(gCamera, kEdsPropID_Record, 0, sizeof(record_start), &record_start);
    }
       
    // Check error 
    if (err != EDS_ERR_OK) { 
        printf("Error starting movie EdsError: %d\n", err);
        recordingActive = false; 
        movElapsedTime = 0.0;
        return;
    }
    
    //set recording active flag
    recordingActive = true; 

    // Log Start Movie elapsed time stopwatch start time
    movStartClock = clock();
    movElapsedTime = 0.0; 

    return;  
}

/*------------------------------------------------------------------------------
* Function:   cmd_stopMovie
* Description: Stops movie recording and turns off movie mode on the camera.
* Parameters: None
* Returns:    None
* Notes:      - Sends the record stop property, then disables the movie switch if
*               it is set. Turning off movie mode may trigger a file-creation
*               event (kEdsObjectEvent_DirItemCreated) which is handled by the
*               object-event callback.
*             - Resets the internal recordingActive flag.
* --------------------------------------------------------------------------*/
void cmd_stopMovie(void)
{

    if (!isSDKInitialized || !isSessionOpen || gCamera == NULL)
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "SDK not initialized or session not open.");
    }

    if (!recordingActive)
    {
        return;
    }

    EdsError err = EDS_ERR_OK;

    // Stop movie recording
    EdsUInt32 record_stop = 0; 
    err = EdsSetPropertyData(gCamera, kEdsPropID_Record, 0, sizeof(record_stop), &record_stop);
              
    // Check Error
    if (err != EDS_ERR_OK)
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
            "Error stopping movie. EdsError: %d", (int)err);
    }

	// Turn off movie mode
    // Check Movie mode
    //EdsUInt32 movieMode;
    //if (err == EDS_ERR_OK)
    //{
    //    err = EdsGetPropertyData(gCamera, kEdsPropID_FixedMovie, 0, sizeof(movieMode), &movieMode);
    //}
    //if (err != EDS_ERR_OK) { printf("Error getting movie mode state. EdsError: %d\n", err); }

    //// Set movie mode to off
    //if (err == EDS_ERR_OK)
    //{
    //    if (movieMode == 1)
    //    {
    //        err = EdsSendCommand(gCamera, kEdsCameraCommand_MovieSelectSwOFF, 0); //triggeres event 204 "kEdsObjectEvent_DirItemCreated"
    //    }
    //}
    //if (err != EDS_ERR_OK) { printf("Error turning off movie mode. EdsError: %d\n", err); }

    recordingActive = false;
    movElapsedTime = 0.0;

    return;
}

// Function to get the amount of time spent recording in seconds as type double
double cmd_getMovTime(void)
{
    if (recordingActive) //block acessing movStartClock when it is not initialized
    {
        movElapsedTime = (clock() - movStartClock) / (double) CLOCKS_PER_SEC;
    }
    else
    {
        movElapsedTime = 0.0;
    }

    return movElapsedTime;
}

/*------------------------------------------------------------------------------
* Function:   cmd_getCameraState
* Description: Builds and returns a MATLAB struct containing the current camera
*              state flags used by the MEX wrapper. Each field is a logical
*              scalar representing an internal boolean state.
* Parameters: None
* Returns:    mxArray* - A 1x1 MATLAB struct with fields:
*                - isSDKInitialized
*                - isSessionOpen
*                - liveViewActive
*                - frameSizeKnown
*                - recordingActive
*                - mexLocked
*                - downloadingActive
* Notes:      - The returned mxArray is a freshly-created MATLAB struct and
*                ownership is transferred to the caller. Caller should manage
*                the returned mxArray (e.g., assign to plhs[] or destroy it).
*             - No EDSDK calls are made; this function only reads internal flags.
* --------------------------------------------------------------------------*/
mxArray* cmd_getCameraState(void)
{    
    // create constant character pointer string array to store ouput fieldnames
    const char* fieldnames[] = { "isSDKInitialized", "isSessionOpen","liveViewActive", "frameSizeKnown",
        "recordingActive", "mexLocked", "downloadingActive", "pendingMovieDownload", "shutterSpeeds", 
        "apertureFNumbers","ISOSpeeds", "currentShutterSpeed", "currentAperture","currentISO", "movieModeActive"};

    // Create matlab structure matrix to populate here
    mxArray* camStateStruct = mxCreateStructMatrix(1, 1, 15, fieldnames);

    // Set all the fields
    mxSetField(camStateStruct, 0, "isSDKInitialized", mxCreateLogicalScalar(isSDKInitialized));

    mxSetField(camStateStruct, 0, "isSessionOpen", mxCreateLogicalScalar(isSessionOpen));

    mxSetField(camStateStruct, 0, "liveViewActive", mxCreateLogicalScalar(liveViewActive));

    mxSetField(camStateStruct, 0, "frameSizeKnown", mxCreateLogicalScalar(frameSizeKnown));

    mxSetField(camStateStruct, 0, "recordingActive", mxCreateLogicalScalar(recordingActive));

    mxSetField(camStateStruct, 0, "mexLocked", mxCreateLogicalScalar(mexLocked));

    mxSetField(camStateStruct, 0, "downloadingActive", mxCreateLogicalScalar(downloadingActive));

    mxSetField(camStateStruct, 0, "pendingMovieDownload", mxCreateLogicalScalar(pendingMovieDownload));

    mxSetField(camStateStruct, 0, "movieModeActive", mxCreateLogicalScalar(movieModeActive));

    // Get all the possibe shutter speed values, aperature values, and iso values
    if (isSDKInitialized && isSessionOpen && gCamera != NULL)
    {
        EdsError err = EDS_ERR_OK;

        EdsPropertyDesc AvPropDesc = { 0 };
        EdsPropertyDesc TvPropDesc = { 0 };
        EdsPropertyDesc ISOPropDesc = { 0 };

        err = EdsGetPropertyDesc(gCamera, kEdsPropID_Av, &AvPropDesc);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "error getting kEdsPropID_Av property description, EdsError code = %d \n", (int)err);
        }

        err = EdsGetPropertyDesc(gCamera, kEdsPropID_Tv, &TvPropDesc);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "error getting kEdsPropID_Tv property description, EdsError code = %d \n", (int)err);
        }

        err = EdsGetPropertyDesc(gCamera, kEdsPropID_ISOSpeed, &ISOPropDesc);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "error getting kEdsPropID_ISOSpeed property description, EdsError code = %d \n", (int)err);
        }

        // Allocate matlab arrays for the possible values 
        mxArray* shutter_speeds = mxCreateDoubleMatrix((mwSize)1, (mwSize)TvPropDesc.numElements, mxREAL);
        mxArray* aperture_fnumbers = mxCreateDoubleMatrix((mwSize)1, (mwSize)AvPropDesc.numElements, mxREAL);
        mxArray* ISO_speeds = mxCreateDoubleMatrix((mwSize)1, (mwSize)ISOPropDesc.numElements, mxREAL);

        // Get pointers to the underlying data in the mxarrays
        double* shutter_speeds_array_values = mxGetDoubles(shutter_speeds);
        double* aperture_fnumbers_array_values = mxGetDoubles(aperture_fnumbers);
        double* ISO_speeds_array_values = mxGetDoubles(ISO_speeds);

        // Loop over all the values in the TvPropDesc and convert them to actual shutter speeds.
        // Modify the underlying data of the mxarray 
        for (int i = 0; i < TvPropDesc.numElements; i++)
        {
            shutter_speeds_array_values[i] = eds_Tv_value_to_shutter_speed(TvPropDesc.propDesc[i]);
        }

        for (int j = 0; j < AvPropDesc.numElements; j++)
        {
            aperture_fnumbers_array_values[j] = eds_av_code_to_fnumber(AvPropDesc.propDesc[j]);
        }

        for (int k = 0; k < ISOPropDesc.numElements; k++)
        {
            ISO_speeds_array_values[k] = eds_iso_code_to_value(ISOPropDesc.propDesc[k]);
        }



        // Get the current settings for Shutter speed, aperture, and iso
        EdsUInt32 currentTv;
        EdsUInt32 currentAv;
        EdsUInt32 currentISOSpeed;

        err = EdsGetPropertyData(gCamera, kEdsPropID_Av, 0, sizeof(currentAv), &currentAv);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "error getting kEdsPropID_Av property data EdsError code = %d \n", (int)err);
        }

        err = EdsGetPropertyData(gCamera, kEdsPropID_Tv, 0, sizeof(currentTv), &currentTv);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "error getting kEdsPropID_Tv property data EdsError code = %d \n", (int)err);
        }

        err = EdsGetPropertyData(gCamera, kEdsPropID_ISOSpeed, 0, sizeof(currentISOSpeed), &currentISOSpeed);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "error getting kEdsPropID_ISOSpeed property data EdsError code = %d \n", (int)err);
        }

        // Add arrays to the state struct
        mxSetField(camStateStruct, 0, "shutterSpeeds", shutter_speeds);
        mxSetField(camStateStruct, 0, "apertureFNumbers", aperture_fnumbers);
        mxSetField(camStateStruct, 0, "ISOSpeeds", ISO_speeds);
        mxSetField(camStateStruct, 0, "currentShutterSpeed", mxCreateDoubleScalar(eds_Tv_value_to_shutter_speed(currentTv)));
        mxSetField(camStateStruct, 0, "currentAperture", mxCreateDoubleScalar(eds_av_code_to_fnumber(currentAv)));
        mxSetField(camStateStruct, 0, "currentISO", mxCreateDoubleScalar(eds_iso_code_to_value(currentISOSpeed)));
    }
    else
    {
        // Add arrays to the state struct
        mxSetField(camStateStruct, 0, "shutterSpeeds", mxCreateDoubleScalar(0.0));
        mxSetField(camStateStruct, 0, "apertureFNumbers", mxCreateDoubleScalar(0.0));
        mxSetField(camStateStruct, 0, "ISOSpeeds", mxCreateDoubleScalar(0.0));
        mxSetField(camStateStruct, 0, "currentShutterSpeed", mxCreateDoubleScalar(0.0));
        mxSetField(camStateStruct, 0, "currentAperture", mxCreateDoubleScalar(0.0));
        mxSetField(camStateStruct, 0, "currentISO", mxCreateDoubleScalar(0.0));
    }

    return camStateStruct;
}

/*------------------------------------------------------------------------------
* Function:   cmd_setTv
* Description: Set the camera shutter speed (Tv) for the supported EOS 5D Mark III
*              by getting the availible values and setting them using a lookup table.
* Parameters: double TvDouble
*              - A real scalar matching one of the supported shutter speed values
*                exactly. Supported values: see eds_Tv_value_to_shutter_speed
* Returns:    None
* Notes:      - Expects an active camera session (global `gCamera` should be
*                valid). The function does not open/close sessions.
*             - If the input value does not match an entry in the lookup
*               table, the function raises a MATLAB error via
*               `mexErrMsgIdAndTxt`.
*             - On failure to set the camera property the function prints an
*               error message with the EDSDK error code (does not raise MATLAB
*               error for property write failures).
* --------------------------------------------------------------------------*/
void cmd_setTv(double TvDouble)
{
    if (isSDKInitialized && isSessionOpen && gCamera != NULL) {
        EdsError err = EDS_ERR_OK;

        // Get possible shutter speed(Tv) values
        EdsPropertyDesc TvPropDesc = { 0 };

        err = EdsGetPropertyDesc(gCamera, kEdsPropID_Tv, &TvPropDesc);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "error getting kEdsPropID_Tv property description, EdsError code = %d \n",(int)err);
        }

        // Loop over all the possible values and determine if the selected value can be set.
        // 
        //Create index variable for correct aperture
        size_t TvIdx = TvPropDesc.numElements + 1;

        // Loop over apeature values to find the index where the aperture value matches the input aperture
        for (int i = 0; i < TvPropDesc.numElements; i++)
        {
            // Get the corresponding double aperature value for the current possible setting
            double current_shutter_speed_seconds = eds_Tv_value_to_shutter_speed(TvPropDesc.propDesc[i]);

            if (current_shutter_speed_seconds == TvDouble)
            {
                TvIdx = i;
                break; //exit loop after setting is found
            }
        }

        if (TvIdx > TvPropDesc.numElements)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:TvError",
                "Input Tv not availible to be set");
        }

        // Get the property code for the desired shutter speed 
        EdsUInt32 Tv_prop_val = TvPropDesc.propDesc[TvIdx];

        // Set the property 
        err = EdsSetPropertyData(gCamera, kEdsPropID_Tv, 0, sizeof(Tv_prop_val), &Tv_prop_val);

        if (err != EDS_ERR_OK) { printf("Error setting property kEdsPropID_Tv. EdsError: %d", err); }
    }
    else
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError", "SDK not initialized or session not open.");
    }
}

/*------------------------------------------------------------------------------
* Function:   cmd_setAv
* Description: Set the camera aperture (Av) for the supported EOS 5D Mark III
*              by mapping a numeric aperture value to the camera-specific
*              Eds property code contained in the `EOS5DMkIIIAv` lookup table.
* Parameters: double apertureDouble
*              - A real scalar matching one of the supported aperture values
*                exactly. Supported values:
*                2.8, 3.2, 3.5, 4.0, 4.5, 5.0, 5.6, 6.3, 7.1, 8.0, 9.0,
*                10, 11, 13, 14, 16, 18, 20, 22, 25, 29, 32
* Returns:    None
* Notes:      - Expects an active camera session (global `gCamera` should be
*                valid). The function does not open/close sessions.
*             - If the input value does not match an entry in the lookup
*               table, the function raises a MATLAB error via
*               `mexErrMsgIdAndTxt`.
*             - On failure to set the camera property the function prints an
*               error message with the EDSDK error code (does not raise MATLAB
*               error for property write failures).
* --------------------------------------------------------------------------*/
void cmd_setAv(double apertureDouble)
{
    if (isSDKInitialized && isSessionOpen && gCamera != NULL) {
        EdsError err = EDS_ERR_OK;

        // Get possible shutter speed(Tv) values
        EdsPropertyDesc AvPropDesc = { 0 };

        err = EdsGetPropertyDesc(gCamera, kEdsPropID_Av, &AvPropDesc);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "error getting kEdsPropID_Av property description, EdsError code = %d \n", (int)err);
        }

        // Loop over all the possible values and determine if the selected value can be set.
        // 
        //Create index variable for correct aperture
        size_t AvIdx = AvPropDesc.numElements + 1;

        // Loop over apeature values to find the index where the aperture value matches the input aperture
        for (int i = 0; i < AvPropDesc.numElements; i++)
        {
            // Get the corresponding double aperature value for the current possible setting
            double current_aperture = eds_av_code_to_fnumber(AvPropDesc.propDesc[i]);

            if (current_aperture == apertureDouble)
            {
                AvIdx = i;
                break; //exit loop after setting is found
            }
        }

        if (AvIdx > AvPropDesc.numElements)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:AvError",
                "Input Av not availible to be set");
        }

        // Get the property code for the desired shutter speed 
        EdsUInt32 Av_prop_val = AvPropDesc.propDesc[AvIdx];

        // Set the property 
        err = EdsSetPropertyData(gCamera, kEdsPropID_Av, 0, sizeof(Av_prop_val), &Av_prop_val);

        if (err != EDS_ERR_OK) { printf("Error setting property kEdsPropID_Av. EdsError: %d", err); }
    }
    else
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError", "SDK not initialized or session not open.");
    }
}


/*------------------------------------------------------------------------------
* Function:   cmd_setISO
* Description: Set the camera ISO speed for the supported EOS 5D Mark III by
*              mapping a numeric ISO value to the camera-specific Eds property
*              code contained in the `EOS5DMkIII_iso` lookup table.
* Parameters: double isoDouble
*              - A real scalar matching one of the supported ISO values exactly.
*                Supported values:
*                -1 (AUTO), 100, 125, 160, 200, 250, 320, 400, 500, 640, 800,
*                1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400, 8000,
*                10000, 12800, 16000, 20000, 25600
* Returns:    None
* Notes:      - Expects an active camera session (global `gCamera` should be valid).
*             - If the input value does not match an entry in the lookup table,
*               the function raises a MATLAB error via `mexErrMsgIdAndTxt`.
*             - On failure to write the property the function prints the EDSDK
*               error code (does not raise a MATLAB error for property write failures).
* --------------------------------------------------------------------------*/
void cmd_setISO(double isoDouble)
{
    if (isSDKInitialized && isSessionOpen && gCamera != NULL) {
        EdsError err = EDS_ERR_OK;

        // Get possible shutter speed(Tv) values
        EdsPropertyDesc ISOPropDesc = { 0 };

        err = EdsGetPropertyDesc(gCamera, kEdsPropID_ISOSpeed, &ISOPropDesc);
        if (err != EDS_ERR_OK)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError",
                "error getting kEdsPropID_ISOSpeed property description, EdsError code = %d \n", (int)err);
        }

        //Create index variable for correct aperture
        size_t isoIdx = ISOPropDesc.numElements + 1;

        // Loop over apeature values to find the index where the aperture value matches the input aperture
        for (int i = 0; i < ISOPropDesc.numElements; i++)
        {
            double current_iso = eds_iso_code_to_value(ISOPropDesc.propDesc[i]);

            if (current_iso == isoDouble)
            {
                isoIdx = i;
                break; //exit loop after setting is found
            }
        }

        if (isoIdx > ISOPropDesc.numElements)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:ISOError",
                "Input ISO not availible to be set");
        }

        // Get the property code for the desired aperture
        EdsUInt32 isoSpeed_prop_val = ISOPropDesc.propDesc[isoIdx];

        // Set the property 
        err = EdsSetPropertyData(gCamera, kEdsPropID_ISOSpeed, 0, sizeof(isoSpeed_prop_val), &isoSpeed_prop_val);

        if (err != EDS_ERR_OK) { printf("Error setting property kEdsPropID_ISOSpeed. EdsError: %d", err); }
    }
    else
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:EDSDKError", "SDK not initialized or session not open.");
    }
}

// The gateway function
/*------------------------------------------------------------------------------
* Function:   mexFunction
* Description: Entry point for the MATLAB MEX function. Dispatches commands to the appropriate functions.
* Parameters: int nlhs - Number of left-hand side (output) arguments.
* 		   mxArray* plhs[] - Array of left-hand side (output) arguments.
* Returns:    None
* --------------------------------------------------------------------------*/
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    if (shutDownRequested) // Flag that signals if the camera is disconnect from computer somehow
    {
        
        cleanup(EDS_ERR_OK); // perform remaining cleanup, clean mex in main matlab thread
        shutDownRequested = false;
        mexShutDownHandled = true; 
        printf("Shutdown request caught, mex unlocked. \n");

        return;
    }

    // Populate cpu clock on access, Set clock
    //movStartClock = clock();

	char command[64]; // buffer to hold command
	//int status = 0; // status to return

	// Check number of inputs
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("edsdk_mex_c:InputError",
            "Atleast one input argument required.");
    }

    // Check first input type
    if (!mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("edsdk_mex_c:TypeError",
            "First input must be a command string (pass as 'command' in matlab with single quotes).");
    }

    // Convert matlab string to C string
    if (mxGetString(prhs[0], command, sizeof(command)) != 0) {
        mexErrMsgIdAndTxt("edsdk_mex_c:ConversionError",
            "Command string too long.");
    }

    if (strcmp(command, "setMovieMode") == 0)
    {
        if (nrhs != 2)
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:InputError",
                "setMovieMode requires exactly 2 inputs: edsdk_mex('setMovieMode', true/false).");
        }

        if (!mxIsScalar(prhs[1]) || mxIsComplex(prhs[1]) ||
            !(mxIsLogical(prhs[1]) || mxIsNumeric(prhs[1])))
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:TypeError",
                "setMovieMode second argument must be logical or numeric scalar.");
        }

        bool movieModeOn = mxGetScalar(prhs[1]) != 0.0;
        cmd_setMovieMode(movieModeOn);
        return;
    }

    // Check for command with two input arguemnts
    if (strcmp(command, "setAv") == 0 || strcmp(command, "setISO") == 0 || strcmp(command, "setTv") == 0)
    {
        // Check if number of arugments is 2
        if (nrhs != 2) 
        {
            mexErrMsgIdAndTxt("edsdk_mex.c:InputError", "setAv, setISO, setTv, and setMovieMode requires exactly 2 inputs: edsdk_mex('setAv', apertureValue) or edsdk_mex('setISO', isoValue).");
        }

        // Check if input is double scalar
        if (!mxIsScalar(prhs[1]) || !mxIsNumeric(prhs[1]) || mxIsComplex(prhs[1]))
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:TypeError",
                "setAv, setTv, or setISO second argument must be a real numeric scalar (e.g. 5.6).");
        }

        if (strcmp(command, "setAv") == 0)
        {
            // convert matlab mxArray object pointer to C double
            double aperatureDouble = mxGetScalar(prhs[1]);

            cmd_setAv(aperatureDouble);
            return;
        }
        else if (strcmp(command, "setISO") == 0)
        {
            
            double isoDouble = mxGetScalar(prhs[1]);

            cmd_setISO(isoDouble);
            return;
        }
        else if (strcmp(command, "setTv") == 0)
        {
            double TvDouble = mxGetScalar(prhs[1]);

            cmd_setTv(TvDouble);
            return;
        }
        else
        {
            mexErrMsgIdAndTxt("edsdk_mex_c:UnknownCommand",
                "Unknown command: %s", command);
        }
        
    }    

	// Dispatch based on command
    if (strcmp(command, "init") == 0)
    {
        cmd_init();
    }
    else if (strcmp(command,"terminate") == 0)
    {
        cmd_terminate();
    }
    else if (strcmp(command,"takePhoto") == 0)
    {
        cmd_takePhoto();
	}
    else if (strcmp(command,"startLiveView") == 0)
    {
        cmd_startLiveView();        
    }
    else if (strcmp(command, "stopLiveView") == 0)
    {
        cmd_stopLiveView();        
	}
    else if(strcmp(command, "getFrame") == 0)
    {
        if (nlhs >= 1)
        {
            mxArray* frame = cmd_getFrame();          // downloads + decodes into mxImage
            plhs[0] = mxDuplicateArray(frame);        // return a MATLAB-owned copy
            // Alternatively: plhs[0] = mxDuplicateArray(mxImage);
        }        
	}
    else if(strcmp(command, "startMovie") == 0)
    {
        cmd_startMovie();
    }
    else if(strcmp(command, "stopMovie") == 0)
    {
        cmd_stopMovie();
	}
    else if (strcmp(command, "getState") == 0)
    {
        plhs[0] = cmd_getCameraState();
    }
    else if (strcmp(command, "getMovTime") == 0)
    {
        double movElapsedTime_output = cmd_getMovTime();

        plhs[0] = mxCreateDoubleScalar(movElapsedTime_output);
    }
    else if (strcmp(command, "downloadMovie") == 0)
    {
        cmd_downloadMovie();
    }
    else
    {
        mexErrMsgIdAndTxt("edsdk_mex_c:UnknownCommand",
            "Unknown command: %s", command);
	}

    /*
    // Set output if requested
    if (nlhs >= 1) {
        plhs[0] = mxCreateDoubleScalar((double)status);
    }
    */
}

