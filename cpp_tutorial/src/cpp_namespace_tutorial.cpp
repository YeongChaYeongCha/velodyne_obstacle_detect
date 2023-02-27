#include <iostream>

int n; //어떤 namespace에도 속하지 않는다.=>전역변수
void set(){
	::n=10; //::n=>명시적 전역변수
}
/*
namespace doodle{
	int n;
	void set(){
		doodle::n=20; //n과 같이 특정 namespace 안에서 소속을 가르키지 않았을 경우에는 속한 namespace 안의 변수이다.
	}
}

namespace google{
	int n;
	void set(){
		google::n=30;
	}
}
*/

namespace doodle{
	//int n;
	void set();/*{
		n=20; //n과 같이 특정 namespace 안에서 소속을 가르키지 않았을 경우에는 속한 namespace 안의 변수이다.
	}*/
	namespace google{
		//int n;
		void set();/*{
			n=30;
		}*/
		int n;
	}
	int n;
}

namespace ns{
	int n;
	void set(); //함수 선언 후 40~43줄과 같이 표기해도 괜찮다.
}

using namespace std;
int main(int argc, char** argv){
	using namespace doodle;
	/*
	::set();
	doodle::set();
	google::set();
	ns::set();
	 
	cout<<::n<<endl;
	cout<<doodle::n<<endl;
	cout<<google::n<<endl;
	cout<<ns::n<<endl;
	*/
	::set();
	doodle::set();
	google::set();
	ns::set();
	
	cout<<::n<<endl;
	cout<<doodle::n<<endl;
	cout<<doodle::google::n<<endl;
	cout<<ns::n<<endl;
}

void doodle::set(){
	n=20;
}

void doodle::google::set(){
	n=30;
}

void ns::set(){
	ns::n=40;
}
