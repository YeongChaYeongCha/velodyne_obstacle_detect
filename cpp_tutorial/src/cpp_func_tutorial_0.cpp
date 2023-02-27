/*
 * overload : 다중정의
 */
#include <iostream>
/*
  // 방법 1
void swap(int *a, int *b){
	int tmp=*a;
	*a=*b;
	*b=tmp;
}
*/

 // 방법 2
void swap(int &a, int &b){
	int tmp=a;
	a=b;
	b=tmp;
}

void swap(double &a, double &b){
	double tmp=a;
	a=b;
	b=tmp;
}

void swap(int* (&a), int* (&b)){
	int *tmp=a;
	a=b;
	b=tmp;
}

int main(int argc, char **argv){
	int num1=20, num2=30;
	double dnum1=2.222, dnum2=3.333;
	int *pnum1=&num1, *pnum2=&num2;
	
	//swap(&num1, &num2); // 방법 1
	swap(num1, num2); //방법 2
	swap(dnum1, dnum2);
	swap(pnum1, pnum2);
	std::cout<<num1<<" | "<<num2<<std::endl;
	std::cout<<dnum1<<" | "<<dnum2<<std::endl;
	std::cout<<*pnum1<<" | "<<*pnum2<<std::endl;
}
