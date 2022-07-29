#include<stdio.h>

unsigned char FS_Command_CreateChecksum(unsigned char Data[], unsigned char size);
unsigned char crcGen[11] = {0xaa,4,117,0,0,0,190,166,36,0,100};
unsigned char temp = 0;


unsigned char crcTest[15] = {0x55,0xaa,0x0d,2,0,6,1,0,0,0,0x0a,0x8c,0x1d,0x7e,0x64};

unsigned char dataIn [32] = {0};

int main(void){
	
		unsigned char lengt;
		while(1){
			printf("\n-------------------------------------\nlengt:");
			scanf("%d",&lengt);
			for(unsigned char i=0;i<lengt;i++){
				printf("byte%d:",i);
				scanf("%x",&dataIn[i]);
				printf("\n");
			}
				printf("%x",FS_Command_CreateChecksum(dataIn,lengt));
		}
	
	//printf("%d",FS_Command_CreateChecksum(crcTest,15));
	return 0;
}

unsigned char FS_Command_CreateChecksum(unsigned char Data[], unsigned char size)
{
	  unsigned char BufferCounter;
	  unsigned char TempCreateCheckSum = 0;
	  for (BufferCounter = 0; BufferCounter < size; BufferCounter++)
	  {			
			TempCreateCheckSum ^= Data[BufferCounter];
	  }
	  return TempCreateCheckSum;
}
