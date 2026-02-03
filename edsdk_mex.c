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
*/

//Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <windows.h>
#include <string.h>
#include <stdbool.h>

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
static volatile bool downloadingActive = false; //bool to flag active download, volatile to allow for multiple threads to access it

// Shutdown Event handling (Need to clear mex in main thread)
static volatile bool shutDownRequested = false; // Flag to queue a cleanup on shutdown
static volatile bool mexShutDownHandled = false; // Flag to indicate if the cleanup (called from shutdown has completed)

/*
// Define structure type for the camera state
struct cameraState {
    bool isSDKInitialized;
    bool isSessionOpen;
    bool liveViewActive;
    bool frameSizeKnown;
    bool recordingActive;
    bool mexLocked;
    bool downloadingActive;
};*/

// camera lock (serialize camera operations when downloading files)
//static CRITICAL_SECTION cameraCS;
//static bool cameraCSInitialized = false;

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
    printf("ObjectEvent: 0x%08X\n", event);
    if (event == kEdsObjectEvent_DirItemRequestTransfer || event == kEdsObjectEvent_DirItemCreated)
    {
        EdsStreamRef stream = NULL;
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

        EdsChar uniqueName[256];

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
            err = EdsDownload(object, dirItemInfo.size, stream);
            downloadingActive = true;
        }
        if (err != EDS_ERR_OK) { printf("Error downloading file. EdsError %d \n", err); }

        if (err == EDS_ERR_OK)
        {
            err = EdsDownloadComplete(object);
            downloadingActive = false;
        }        

        if (err == EDS_ERR_OK)
        {
            printf("Downloaded file: %s\n", uniqueName);            
        }        
        eventHasFired = true;
        if(stream) EdsRelease(stream);
        stream = NULL;
    }
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
    printf("StateEvent: 0x%08X\n", event);

    if (event == kEdsStateEvent_Shutdown) //if camera is disconnected from computer
    {
        // Reset flags
        eventHasFired = true; 
        downloadingActive = false;
        liveViewActive = false; 
        recordingActive = false; 
        isSessionOpen = false;       
        

        //Clear EDS related objects
        if (gEvfImage) { EdsRelease(gEvfImage); gEvfImage = NULL; }
        if (gEvfStream) { EdsRelease(gEvfStream); gEvfStream = NULL; }
        if (gCamera) { EdsRelease(gCamera); gCamera = NULL; }
        if (gCameraList) { EdsRelease(gCameraList); gCameraList = NULL; }

        //Clear turbo jpeg stuff 
        if (tj) { tjDestroy(tj); tj = NULL; }
                            
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
        cleanup(err);
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
        return mxImage;
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
    EdsUInt32 saveTo = kEdsSaveTo_Camera;
    err = EdsSetPropertyData(gCamera, kEdsPropID_SaveTo, 0, sizeof(saveTo), &saveTo);
    if (err != EDS_ERR_OK) { printf("Error seting save to location to camera. EdsError: %d\n", err); }
    
    // Check Movie mode
    EdsUInt32 movieMode;
    if (err == EDS_ERR_OK) 
    {
        err = EdsGetPropertyData(gCamera, kEdsPropID_FixedMovie, 0, sizeof(movieMode), &movieMode);
    }
    if (err != EDS_ERR_OK) { printf("Error getting movie mode state. EdsError: %d\n", err); }

    // Set movie mode to on
    if (err == EDS_ERR_OK)
    {
        if (movieMode == 0)
        {
            err = EdsSendCommand(gCamera, kEdsCameraCommand_MovieSelectSwON, 0);
        }
    }
    if (err != EDS_ERR_OK) { printf("Error turning on movie mode. EdsError: %d\n", err); }    
    
    // Begin movie shooting
    EdsUInt32 record_start = 4;
    if (err == EDS_ERR_OK)
    {
        err = EdsSetPropertyData(gCamera, kEdsPropID_Record, 0, sizeof(record_start), &record_start);
    }
       
    // Check error 
    if (err != EDS_ERR_OK) { printf("Error starting movie EdsError: %d\n", err); }
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
    EdsError err = EDS_ERR_OK;

    if (gCamera)
    {
        // Stop movie recording
        EdsUInt32 record_stop = 0; 
        err = EdsSetPropertyData(gCamera, kEdsPropID_Record, 0, sizeof(record_stop), &record_stop);
              
        // Check Error
        if (err != EDS_ERR_OK) { printf("Error stopping movie EdsError: %d\n", err); }          

		// Turn off movie mode
        // Check Movie mode
        EdsUInt32 movieMode;
        if (err == EDS_ERR_OK)
        {
            err = EdsGetPropertyData(gCamera, kEdsPropID_FixedMovie, 0, sizeof(movieMode), &movieMode);
        }
        if (err != EDS_ERR_OK) { printf("Error getting movie mode state. EdsError: %d\n", err); }

        // Set movie mode to off
        if (err == EDS_ERR_OK)
        {
            if (movieMode == 1)
            {
                err = EdsSendCommand(gCamera, kEdsCameraCommand_MovieSelectSwOFF, 0); //triggeres event 204 "kEdsObjectEvent_DirItemCreated"
            }
        }
        if (err != EDS_ERR_OK) { printf("Error turning off movie mode. EdsError: %d\n", err); }
    }

    recordingActive = false;
    return;
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
        "recordingActive", "mexLocked", "downloadingActive" };

    // Create matlab structure matrix to populate here
    mxArray* camStateStruct = mxCreateStructMatrix(1, 1, 7, fieldnames);

    // Set all the fields
    mxSetField(camStateStruct, 0, "isSDKInitialized", mxCreateLogicalScalar(isSDKInitialized));

    mxSetField(camStateStruct, 0, "isSessionOpen", mxCreateLogicalScalar(isSessionOpen));

    mxSetField(camStateStruct, 0, "liveViewActive", mxCreateLogicalScalar(liveViewActive));

    mxSetField(camStateStruct, 0, "frameSizeKnown", mxCreateLogicalScalar(frameSizeKnown));

    mxSetField(camStateStruct, 0, "recordingActive", mxCreateLogicalScalar(recordingActive));

    mxSetField(camStateStruct, 0, "mexLocked", mxCreateLogicalScalar(mexLocked));

    mxSetField(camStateStruct, 0, "downloadingActive", mxCreateLogicalScalar(downloadingActive));

    return camStateStruct;
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

	char command[64]; // buffer to hold command
	//int status = 0; // status to return

	// Check number of inputs
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("edsdk_mex_c:InputError",
            "Exactly one input argument required.");
    }

    // Check input type
    if (!mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("edsdk_mex_c:TypeError",
            "Input must be a command string (pass as 'command' in matlab).");
    }

    // Convert matlab string to C string
    if (mxGetString(prhs[0], command, sizeof(command)) != 0) {
        mexErrMsgIdAndTxt("edsdk_mex_c:ConversionError",
            "Command string too long.");
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

