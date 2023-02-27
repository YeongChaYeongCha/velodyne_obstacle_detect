/*
class : 자료저장 + 자료처리 = 변수 + 함수
class(타입) : 특정 용도 수행을 위해 변수와 함수를 모아둔 틀(자료)
객체(object) : class(틀)을 이용하여 찍어낸 개체(변수, 메모리 상 공간)형
*/
#include <iostream>

using namespace std;

//접근제어지시자 : private, protected, public
struct TV{ 
//struct는 접근제어지시자를 명시하지 않으면 기본적으로 public으로 선언되며 class는 접근제어지시자를 명시하지 않으면 기본적으로 private로 선언된다. 
private: //구조체 내에서 접근가능
	bool powerOn;
	int channel;
	int volume;
	
public: //구조체 밖에서 접근가능
	void on(){
		powerOn=true;
		cout<<"TV가 커졌습니다."<<endl;
	}
	void off(){
		powerOn=false;
		cout<<"TV가 꺼졌습니다."<<endl;
	}
	void setChannel(int cnl){
		if(cnl>=1 && cnl<=999){
			channel=cnl;
			cout<<"채널을 "<<cnl<<"(으)로 변경했습니다."<<endl;
		}
	}
	void setVolume(int vol){
		if(vol>=0 && vol<=100){
			volume=vol;
			cout<<"볼륨을 "<<vol<<"(으)로 변경했습니다."<<endl;
		}
	}
};

int main(int argc, char** argv){
	TV lg;
	/* private 내에서 powerOn, channel, volume이 선언되어 있기 때문에 main함수 내에서 접근불가.
	lg.powerOn=true;
	lg.channel=10;
	lg.setVolume(50);
	*/
	lg.on();
	lg.setChannel(10);
	lg.setVolume(50);
	lg.setVolume(-73);
}
