#include <iostream>

int inventory[64]={0};
int score=0;
/*
void getItem(int itemId){
	inventory[itemId]++;
}

void getItem(int itemId, int cnt){
	inventory[itemId]+=cnt;
}

void getItem(int itemId, int cnt, int sc){
	inventory[itemId]+=cnt;
	score+=sc;
}
*/
void getItem(int itemId, int cnt=1, int sc=0){ //cnt, sc의 default값을 설정 | default값 매개변수는 항상 오른쪽부터
	inventory[itemId]+=cnt;
	score+=sc;
}

using namespace std;
int main(int argc, char** argv){
	getItem(6, 5);
	getItem(3, 2);
	getItem(3);
	getItem(11, 34, 7000);
	
	cout<<score<<endl;
	for(int i=0; i<16; i++){
		cout<<inventory[i]<<" | ";
	}
	cout<<endl;
}
