#include<stdio.h>
#include<drawlib.h>
#include<time.h>
FILE *fp;
FILE *fp2;

int main (){
	char fname[50];
	int n=0;
	int h=0;
	int i,j;
	double Pos[500][3];
	double SEN[50][3];
	while(1){
		printf("txtファイルを入力=>");
		scanf("%s", fname); 
		fp=fopen(fname,"r");
		if(fp!=NULL){
			break;
		}
		printf("txtファイルの中身がありません\n");

	}
	
	fp2=fopen("KAMI.txt","r");
	//5, 5, 635, 475
		
	while ( ! feof(fp) && n < 500) {
		for(i = 0; i < 3;i++){
			fscanf(fp, "%lf", &(Pos[n][i]));
			Pos[n][i] = Pos[n][i]*1000;
			printf("%lf", Pos[n][i]);
		}
		n++;
		printf("\n");
	}
	printf("実行１\n");	
	
	dl_initialize(1.0);
	dl_clear(DL_C("white"));
	printf("実行２\n");
	
	while(1){
		if(h < n){
			printf("%d\n", h);
			if(Pos[h][2]  == 100){
				if(Pos[h+1][2] == 100){
					 dl_line(250-(int)Pos[h][1]*1.2,480-(int)Pos[h][0]*1.2,250-(int)Pos[h+1][1]*1.2, 480-Pos[h+1][0]*1.2, dl_color_from_name("black"), 6);
					dl_line(250-(int)Pos[h][1],400-(int)Pos[h][0],250-(int)Pos[h+1][1], 400-Pos[h+1][0], dl_color_from_name("black"), 6);
					printf("x %lf y %lf \n x %lf y %lf \n", Pos[h][0],Pos[h][1],Pos[h+1][0],Pos[h+1][1]);
				}	
			}
		h++;
		}
	}

}
