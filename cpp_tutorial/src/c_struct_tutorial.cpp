#include <iostream>

//typedef struct{int x, y;} Point;
/*
struct Point {
	int x, y;
	char c;
	double d;
};
*/
struct ProductInfo{
	int num; //4byte
	char name[100]; //100byte
	int cost; //4byte
};
/*
void productSale(ProductInfo p, int percent){ //call-by-value이기 때문에 실제 값이 변경되지 않는다.
	p.cost-=p.cost*percent/100;
}
*/

struct Time{
	int h, m, s;
	
	int totalSec(){ //멤버 메써드
		return 3600*h+60*m+s;
	}
};

struct Point{
	int x, y;
	
	void moveRight(){ x++; }
	void moveLeft(){ x--; }
	void moveUp(){ y++; }
	void moveDown(){ y--; }
};

void productSale(ProductInfo *p, int percent){ 
	p->cost-=p->cost*percent/100;
}

void productSwap(ProductInfo *a, ProductInfo *b){
	ProductInfo tmp=*a;
	*a=*b;
	*b=tmp;
}

using namespace std;
int main(int argc, char** argv){
	/*
	Point p;
	p.x=3;
	p.y=4;
	
	std::cout<<p.x<<" | "<<p.y<<std::endl;
	std::cout<<sizeof(p)<<std::endl; //p라는 변수는 정수형인 x와 y 2개로 이루어져 있기 때문에 총 8byte의 크기를 가진다.
	
	ProductInfo myProduct={4797283, "제주 한라봉", 20000};
	ProductInfo otherProduct={5797283, "성주 꿀참외", 10000};
	*/
	/*
	cout<<"상품번호 : "<<myProduct.num<<endl;
	cout<<"상품명 : "<<myProduct.name<<endl;
	cout<<"가격 : "<<myProduct.cost<<endl;
	cout<<sizeof(myProduct)<<"byte"<<endl;
	
	//주소값 출력
	cout<<&myProduct<<endl;
	cout<<"상품번호 : "<<&myProduct.num<<endl; //상품번호의 메모리 주소에 +4byte = 상품명 메모리 주소
	cout<<"상품명 : "<<&myProduct.name<<endl; //상품명의 메모리 주소에 +100byte = 가격 메모리 주소
	cout<<"가격 : "<<&myProduct.cost<<endl; 
	
	ProductInfo *ptr_product=&myProduct;
	*/
	/*
	cout<<"상품번호 : "<<(*ptr_product).num<<endl;
	cout<<"상품명 : "<<(*ptr_product).name<<endl;
	cout<<"가격 : "<<(*ptr_product).cost<<endl;
	
	cout<<"상품번호 : "<<ptr_product->num<<endl; // (*ptr_product).num과 ptr_product->num은 같은 표현
	cout<<"상품명 : "<<ptr_product->name<<endl; // (*a).b == a->b
	cout<<"가격 : "<<ptr_product->cost<<endl;
	*/
	//productSale(&myProduct, 10);
	/*
	productSwap(&myProduct, &otherProduct);
	
	cout<<"상품번호 : "<<myProduct.num<<endl;
	cout<<"상품명 : "<<myProduct.name<<endl;
	cout<<"가격 : "<<myProduct.cost<<endl;
	
	cout<<endl<<"상품번호 : "<<otherProduct.num<<endl;
	cout<<"상품명 : "<<otherProduct.name<<endl;
	cout<<"가격 : "<<otherProduct.cost<<endl;
	*/
	
	//Time t={1, 22, 48}; //1시간=60분 | 1분=60초 | --> 1*3600 + 22*60 + 48
	//cout<<t.totalSec()<<endl;
	
	Point p={2, 5};
	p.moveDown();
	
	cout<<p.x<<" | "<<p.y<<endl;
}
