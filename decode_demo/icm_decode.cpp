/* *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ *
* File: icm_decode.cpp
* Description : Main application for decoding  IMU data from BMP images
* Author : TSTC
* Date : 2026 - 04 - 14
* Version : 10.00.00
​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ */

#include <iostream>
#include <string>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "./inc/icm42688_decode.h"


#define D_GROUP_DT_SIZE 16

#pragma pack(push, 1)

/* *​
* @brief BMP file header structure
* Contains both bitmap file header and bitmap information header
*/
typedef struct {
        /* Bitmap File Header */
        uint8_t  file_type[2]; /* *​ < Bitmap type : 'BM' -> 0x4D42 */
        uint32_t file_size; /* *​ < Complete file size in bytes */
        uint16_t reserved1; /* *​ < Reserved field */
        uint16_t reserved2; /* *​ < Reserved field */
        uint32_t offset; /* *​ < Offset to pixel data */

        /* Bitmap Information Header */
        uint32_t head_size; /* *​ < Information header size */
        uint32_t width; /* *​ < Image width in pixels */
        uint32_t height; /* *​ < Image height in pixels */
        uint16_t bit_planes; /* *​ < Number of color planes */
        uint16_t bits_per_pixel; /* *​ < Bits per pixel */
        uint32_t compression; /* *​ < Compression type */
        uint32_t image_size; /* *​ < Image data size */
        uint32_t h_res; /* *​ < Horizontal resolution */
        uint32_t v_res; /* *​ < Vertical resolution */
        uint32_t color_palette; /* *​ < Number of color indexes */
        uint32_t vip_color; /* *​ < Important color indexes */
        uint8_t  rgb_data[1]; /* *​ < Pixel data array */
} bmp_header_t;






#pragma pack(pop)


enum
{
eDev_IMC42688       = 1,
eDev_2          	= 2,
eDev_3    			= 3


};





/***设备类型
eDev_IMC42688           = 1,
eDev_2  		        = 2,
eDev_3    				= 3
* 
**************/

typedef struct sGroupInfo
{
    uint8_t      ProtocolType;       //协议类型
    uint8_t      DeviceType  [5];    //设备类型
    uint8_t      DeviceDtSize[5];
    /* *​
    * @brief Frame exposure time structure
    * Contains exposure start and end timestamps(watch for 32 - bit overflow)
    */
    uint64_t	 uExpTimeStart;	/*Exposure start time in microseconds */
    uint64_t	 uExpTimeEnd;	/*Exposure end time in microseconds */
};





void   F_Group_0_Analysis(uint8_t* pDtU8, sGroupInfo *pHeader)
{
     //旧的协议 0~7 和 8~16 byte都是时间戳传输数据相同的
    if ((pDtU8[0] == pDtU8[8]) && (pDtU8[1] == pDtU8[9]) && (pDtU8[2] == pDtU8[10]) && (pDtU8[3] == pDtU8[11]) &&
        (pDtU8[4] == pDtU8[12]) && (pDtU8[5] == pDtU8[13]) && (pDtU8[6] == pDtU8[14]) && (pDtU8[7] == pDtU8[15]))
    {
        pHeader->ProtocolType = 0;
        memset(pHeader->DeviceType,     0, 0);
        memset(pHeader->DeviceDtSize,   0, 0);
        pHeader->DeviceType[0]      = eDev_IMC42688;
        pHeader->DeviceDtSize[0]    = 11;

        uint32_t    val = 0;

        D_U8_TO_U32(val, pDtU8 + 0);

        pHeader->uExpTimeStart = val;

        D_U8_TO_U32(val, pDtU8 + 4);

        pHeader->uExpTimeEnd = val;

    }
    else
    {
       
        memset(pHeader->DeviceType, 0, 0);
        memset(pHeader->DeviceDtSize, 0, 0);
        pHeader->ProtocolType       = pDtU8[0] >> 4;   //[63~60bit]
        pHeader->DeviceType[0]      = ((pDtU8[0]&0xf) << 4) | ((pDtU8[1] & 0xf0) >> 4);//[59~52bit]
        pHeader->DeviceDtSize[0]    = pDtU8[1]&0xf;   //[51~48bit]

        pHeader->DeviceType[1]      = pDtU8[2];         //[47~40bit]
        pHeader->DeviceDtSize[1]    = pDtU8[3] >> 4;    //[39~36bit]

        pHeader->DeviceType[2]      = ((pDtU8[3] & 0xf) << 4) | ((pDtU8[4] & 0xf0) >> 4);;         //[35~28bit]
        pHeader->DeviceDtSize[2]    = pDtU8[4]&0xf;    //[27~24bit]

        pHeader->DeviceType[3]      = pDtU8[5];         //[23~16bit]
        pHeader->DeviceDtSize[3]    = pDtU8[6] >> 4;    //[15~12bit]

        pHeader->DeviceType[4]      = ((pDtU8[6] & 0xf) << 4) | ((pDtU8[7] & 0xf0) >> 4);;         //[11~4bit]
        pHeader->DeviceDtSize[4]    = pDtU8[7]&0xf;    //[3~0bit]

        uint32_t    val = 0;

        D_U8_TO_U32(val, pDtU8 + 8);

        pHeader->uExpTimeStart  = val;

        D_U8_TO_U32(val, pDtU8 + 12);

        pHeader->uExpTimeEnd    = val;

    }
}


void   F_Group_Dev_Analysis(uint8_t* pDtU8, uint8_t DevType, uint8_t GroupSize)
{
    for (uint32_t index = 0; index < GroupSize; index++)
    {
        switch (DevType)
        {
            case eDev_IMC42688:
            {
                sICM42688_XYZ_float  s_ICM42688;
                F_Analysis_Icm42688( s_ICM42688, pDtU8 + index*16, AFS_4G, GFS_1000DPS);


                fprintf(stdout, "IMC [1/1000(g)] 0x%X AccData X,Y,Z (%0.5f,\t%0.5f,\t%0.5f) GyroData[1/1000(dps)] X,Y,Z(%0.5f,\t%0.5f,\t%0.5f) sampling time[%lu us]\n",
                    index,
                    s_ICM42688.fAccData_X,
                    s_ICM42688.fAccData_Y,
                    s_ICM42688.fAccData_Z,
                    s_ICM42688.fGyroData_X,
                    s_ICM42688.fGyroData_Y,
                    s_ICM42688.fGyroData_Z,
                    s_ICM42688.uTime);

                break;
            }

        }

    }


}   









/* *​
* @brief Initialize BMP header structure
* @param[out] header Pointer to BMP header structure
* @param[in] w Image width
* @param[in] h Image height
* @param[in] cbyte Color bytes per pixel
* @return Total file size
*/
uint32_t init_bmp_header(bmp_header_t * header, int32_t w, int32_t h, const uint8_t cbyte)
{
    /* Clear header memory */
    memset(header, 0, 54);

    /* Set BMP information */
    header->file_type[0] = 'B';
    header->file_type[1] = 'M';
    header->file_size = (w * h * cbyte) + 54;
    header->offset = 54;
    header->head_size = 40;
    header->width = w;
    header->height = h;
    header->bit_planes = 1;
    header->bits_per_pixel = cbyte * 8;
    header->image_size = w * h * cbyte;

    return (w * h * cbyte) + 54;
}

/* *​
* @brief Create file and write binary data
* @param[in] path File path
* @param[in] str Data buffer
* @param[in] size Data size
* @return true if successful, false otherwise
*/
bool CreateFileAndWriteBin(const char* path, uint8_t * str, uint32_t size)
{
    FILE* pfd;
    fopen_s(&pfd, path, "wb+");

    if (pfd == NULL)
    {
        fprintf(stderr, "ERROR: Failed to open file %s\n", path);
        return false;
    }
    else
    {
        fprintf(stderr, "Successfully opened file {%s} size[%d]\n", path, size);
    }

    fwrite(str, 1, size, pfd);
    fflush(pfd);
    fclose(pfd);

    return true;
}

/* *​
* @brief Read binary data from file
* @param[in] fileName File name to read
* @param[out] pSize Pointer to store file size
* @return Pointer to allocated buffer with file data
*/
uint8_t * ReadSrcBin(const char* fileName, uint32_t * pSize)
{
    FILE* fp;
    fopen_s(&fp, fileName, "rb");

    if (!fp)
    {
        fprintf(stderr, "Failed to open %s\n", fileName);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int32_t size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    uint8_t* source_str = (uint8_t*)malloc(size + 1024);
    size_t source_size = 0;

    if (source_str != NULL)
    {
        memset(source_str, 0, size + 1024);
        source_size = fread(source_str, 1, size, fp);
    }

    fclose(fp);

    if (source_size == 0)
    {
        fprintf(stderr, "File read error: %s\n", fileName);
    }

    *pSize = (uint32_t)source_size;
    return source_str;
}

/* *​
* @brief Decode line data from BMP pixel data
* @description:
* Data before and after encoding is over - white data above 220, used for data alignment.
* In encoding, pixel values 0~50 represent 0, 60~220 represent 1.
* @param[in] pBmp BMP pixel data
* @param[in] Vindex Row index(which row)
* @param[in] ColorSize BMP bit depth(24bit = 3 bytes)
* @param[in] HSize BMP line width
* @param[in] USize Encoding size[8X8] = 8
* @param[out] pDtU8 Decoded data buffer
* @return Size of decoded data
*/
uint32_t F_LineDecode(uint8_t* pBmp, uint32_t Vindex, uint32_t ColorSize, uint32_t HSize, uint32_t USize, uint8_t* pDtU8)
{
    uint8_t* pBitDt = new uint8_t[HSize];
    uint32_t uBitSize = 0;
    uint32_t uCind = ColorSize >> 1;
    uint16_t uVal_0 = 50;
    uint16_t uVal_1 = 220;
    uint32_t ind = 0;


    //fprintf(stderr, "%d %d %d\r\n", ColorSize, HSize, Vindex);

    uint8_t* pData = pBmp + (ColorSize * HSize * Vindex);

    if ((pData[0] > uVal_1) && 
        (pData[1] > uVal_1) && 
        (pData[2] > uVal_1) && 
        (pData[3] > uVal_1))
    {

    }
    else
    {
        delete[] pBitDt;
        
        return 0;
    }



    /* Analyze the starting position of the encoding */
    for (ind = 0; ind < HSize; ind++)
    {
        if (pData[(ind * ColorSize) + uCind] < uVal_1)
        {
            ind = ind + (USize >> 1);
            break;
        }
    }

    /* Extract the encoded data */
    uint32_t uBitIndex = 0;

    for (; ind < HSize; ind = ind + USize)
    {
        uint16_t uVAL = pData[(ind * ColorSize) + uCind] + pData[((ind + 1) * ColorSize) + uCind];

        if (uVAL < (uVal_0 << 1))
        {
            pBitDt[uBitIndex++] = 0;
        }
        else if (uVAL < (uVal_1 << 1))
        {
            pBitDt[uBitIndex++] = 1;
        }
        else
        {
            uBitSize = uBitIndex;
            break;
        }
    }

    /* Convert bit data to byte data */
    uint32_t uDtSize = uBitSize / 8;

    for (uint32_t uDtind = 0; uDtind < uDtSize; uDtind++)
    {
        uint8_t Val = 0;
        Val |= pBitDt[uDtind * 8 + 0] << 0;
        Val |= pBitDt[uDtind * 8 + 1] << 1;
        Val |= pBitDt[uDtind * 8 + 2] << 2;
        Val |= pBitDt[uDtind * 8 + 3] << 3;
        Val |= pBitDt[uDtind * 8 + 4] << 4;
        Val |= pBitDt[uDtind * 8 + 5] << 5;
        Val |= pBitDt[uDtind * 8 + 6] << 6;
        Val |= pBitDt[uDtind * 8 + 7] << 7;

        pDtU8[uDtind] = Val;
    }

    delete[] pBitDt;
    return uDtSize;
}









//#define D_H_DEBUG





/* *​
* @brief Main application entry point
* @param[in] argc Argument count
* @param[in] argv Argument values
* @return Exit status
*/
int main(int argc, char* argv[])
{
    std::string inputFile = "4000x1200_0_0_10.bmp";  // Default input file

    uint16_t bmpRgbFormat = 1;

    /* Parse command line arguments */
    for (uint32_t i = 0; i < argc; i++)
    {
        fprintf(stderr, "[%d] %s\n", i, argv[i]);
        if (i == 1)
        {
            inputFile = argv[i];
        }
    }

    uint32_t fileSize = 0;
    uint8_t* pBmpData = ReadSrcBin(inputFile.c_str(), &fileSize);

    if (pBmpData == NULL)
    {
        fprintf(stderr, "Failed to open file %s\n", inputFile.c_str());
        exit(EXIT_FAILURE);
    }

    bmp_header_t* pBmp = (bmp_header_t*)pBmpData;
    uint32_t colorSize = pBmp->image_size / (pBmp->width * pBmp->height);

    fprintf(stdout, "Bmp[%d]-HSize[%d]-VSize[%d] Tsize[%d]\n",
        colorSize,
        pBmp->width,
        pBmp->height,
        pBmp->file_size);

    /* Frame data analysis */
//#define LINE_VALID_DATA_SIZE 32


    uint8_t frameBuffer[1024];
    uint32_t frameBufferSize = 0;
    uint8_t* lineData = new uint8_t[pBmp->width];
    memset(frameBuffer, 0, sizeof(frameBuffer));

    /* Read data lines from BMP */
    uint32_t startRowIndex = pBmp->height - 3;//The first encoding line has been corrected to 8X8, with a range of [3-5]
    //Each data group consists of 16 bytes, with a total of 12 groups

    uint32_t    CodeHSize   = 8;
    uint32_t    CodeVSize   = 8;
    uint32_t    lineIndex   = 0;
    uint32_t    TotalSize   = D_GROUP_DT_SIZE;

    for (; lineIndex < TotalSize; lineIndex++)
    {
        int32_t rowIndex = startRowIndex - lineIndex * CodeVSize;
        memset(lineData, 0, pBmp->width);

        if ((rowIndex < 0)||(rowIndex > pBmp->height))
        {
            break;
        }
        uint32_t lineDataSize = F_LineDecode(pBmp->rgb_data, rowIndex, colorSize, pBmp->width, CodeHSize, lineData);
#ifdef D_H_DUBUG
        fprintf(stderr, "F_LineDecode [%d] RowIndex:%d ->[%d]\n", lineIndex, rowIndex, lineDataSize);
        /* Display decoded data */
         /****************logic s************************/

        for (uint32_t index = 0; index < lineDataSize; index++)
        {
            //           if ((index % 4) == 0)
            //           {
            //               fprintf(stdout, "\r\n");
            //           }
            fprintf(stdout, "%02X \t ", lineData[index]);
        }
        fprintf(stdout, "\r\n");
#endif // D_H_DUBUG


        /****************logic e************************/
        memcpy(frameBuffer + frameBufferSize, lineData, lineDataSize);
        frameBufferSize += lineDataSize;
        //fprintf(stdout, "total %02X \t\r\n ", frameBufferSize);
        if (TotalSize <= frameBufferSize) {
            break;
        }
    }
    ///////


    //read group 0
    sGroupInfo  DtHeader;
    F_Group_0_Analysis(frameBuffer, &DtHeader);
    /****************logic s************************/
    uint32_t GroupSize = 1 +
        DtHeader.DeviceDtSize[0] + DtHeader.DeviceDtSize[1] +
        DtHeader.DeviceDtSize[2] + DtHeader.DeviceDtSize[3] +
        DtHeader.DeviceDtSize[4];

    fprintf(stdout, "------------group size %d-----------\r\n ", GroupSize);
    /****************logic e************************/


    TotalSize = D_GROUP_DT_SIZE * GroupSize;


    for (lineIndex++; lineIndex < TotalSize; lineIndex++)
    {
        int32_t rowIndex = startRowIndex - lineIndex * CodeVSize;
        memset(lineData, 0, pBmp->width);

        if ((rowIndex < 0) || (rowIndex > pBmp->height))
        {
            break;
        }
        uint32_t lineDataSize = F_LineDecode(pBmp->rgb_data, rowIndex, colorSize, pBmp->width, CodeHSize, lineData);
#ifdef D_H_DUBUG
        /* Display decoded data */
         /****************logic s************************/
        fprintf(stderr, "F_LineDecode [%d] RowIndex:%d ->[%d]\n", lineIndex, rowIndex, lineDataSize);


        for (uint32_t index = 0; index < lineDataSize; index++)
        {
            //           if ((index % 4) == 0)
            //           {
            //               fprintf(stdout, "\r\n");
            //           }
            fprintf(stdout, "%02X \t ", lineData[index]);
        }
        fprintf(stdout, "\r\n");
#endif // D_H_DUBUG
        /****************logic e************************/
        memcpy(frameBuffer + frameBufferSize, lineData, lineDataSize);
        frameBufferSize += lineDataSize;
        //fprintf(stdout, "total %02X \t\r\n ", frameBufferSize);
        if (TotalSize <= frameBufferSize) { break; }
    }





    /**********************************************************/


    fprintf(stdout, "%02X |\t ", 0);
    for (uint32_t x = 0; x < D_GROUP_DT_SIZE; x++)
    {
        fprintf(stdout, "%02X \t ", x);
    }
    fprintf(stdout, "\t\n----------------------------------------------------\r\n");

    for (uint32_t y = 0; y < GroupSize; y++)
    {
        fprintf(stdout, "%02X |\t ", y);
        for (uint32_t x = 0; x < D_GROUP_DT_SIZE; x++)
        {
            fprintf(stdout, "%02X \t ", frameBuffer[y * D_GROUP_DT_SIZE + x]);
        }
        fprintf(stdout, "\r\n");
    }

    /**********************************************************/





    uint32_t    GroupIndex = 0;

   /* Display results s*/
   fprintf(stdout, "Frame exposure time start:[%lu us] - end:[%lu us] frame exposure[%ldus]\n",
       DtHeader.uExpTimeStart,
       DtHeader.uExpTimeEnd,
       DtHeader.uExpTimeEnd - DtHeader.uExpTimeStart);
   /* Display results e*/
   GroupIndex = GroupIndex + 1;

   F_Group_Dev_Analysis(frameBuffer + GroupIndex * D_GROUP_DT_SIZE, DtHeader.DeviceType[0], DtHeader.DeviceDtSize[0]); GroupIndex = GroupIndex + DtHeader.DeviceDtSize[0];
   F_Group_Dev_Analysis(frameBuffer + GroupIndex * D_GROUP_DT_SIZE, DtHeader.DeviceType[1], DtHeader.DeviceDtSize[1]); GroupIndex = GroupIndex + DtHeader.DeviceDtSize[1];
   F_Group_Dev_Analysis(frameBuffer + GroupIndex * D_GROUP_DT_SIZE, DtHeader.DeviceType[2], DtHeader.DeviceDtSize[2]); GroupIndex = GroupIndex + DtHeader.DeviceDtSize[2];
   F_Group_Dev_Analysis(frameBuffer + GroupIndex * D_GROUP_DT_SIZE, DtHeader.DeviceType[3], DtHeader.DeviceDtSize[3]); GroupIndex = GroupIndex + DtHeader.DeviceDtSize[3];
   F_Group_Dev_Analysis(frameBuffer + GroupIndex * D_GROUP_DT_SIZE, DtHeader.DeviceType[4], DtHeader.DeviceDtSize[4]); GroupIndex = GroupIndex + DtHeader.DeviceDtSize[4];

    delete[] lineData;

    free(pBmpData);
    system("pause");
    return EXIT_SUCCESS;
}