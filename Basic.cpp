#include <bits/stdc++.h>
#include <assert.h>
using namespace std;

/*==========Pointクラス===========================================*/

#define EPS (1e-10)         //非常に小さい値の定義
#define equals(a,b) (fabs((a) - (b)) < EPS )    //double型の絶対値

class Point {
    public:
    double x, y;    //ベクトルを表す点の記録

    Point(double x = 0, double y = 0): x(x), y(y) {}

    //点とベクトルに対して演算をするための演算子の定義
    Point operator + (Point p) { return Point(x + p.x, y + p.y); }
    Point operator - (Point p) { return Point(x - p.x, y - p.y); }
    Point operator * (double a) { return Point(a * x, a * y); }
    Point operator / (double a) { return Point(x / a, y / a); }

    //原点からベクトルを表す点までの距離を返す
    double abs() { return sqrt(norm());}
    double norm() { return x * x + y * y;}

    //比較演算子の定義
    bool operator < (const Point &p) const {
        return x != p.x ? x < p.x : y < p.y;
    }

    bool operator == (const Point &p) const {
        return fabs(x - p.x) < EPS && fabs(y - p.y) < EPS;
    }
};

typedef Point Vector;   //Pointと同じVectorクラスの定義

//a.norm()でもnorm(a)でも使える
double norm(Vector a) {
    return a.x * a.x + a.y * a.y;
}
double abs(Vector a) {
    return sqrt(norm(a));
}


struct Segment {    //線分
    Point p1, p2;
};

typedef Segment Line;   //直線


class Circle {  //円
public:
    Point c;
    double r;
    Circle(Point c = Point(), double r = 0.0): c(c), r(r) {}
};


typedef vector<Point> Polygon;  //多角形


//ベクトルaとbの内積
double dot(Vector a, Vector b) {
    return a.x * b.x + a.y * b.y;
}

//ベクトルaとbの外積
double cross(Vector a, Vector b) {
    return a.x * b.y - a.y * b.x;
}


/*==========直線の直交・平行判定=====================================*/
//直交判定
bool isOrthogonal(Vector a, Vector b) {
    return equals(dot(a, b), 0.0);
    //ベクトルa,bの内積が0だったらa,bは直交
}

bool isOrthogonal(Point a1, Point a2, Point b1, Point b2) {
    return isOrthogonal(a1 - a2, b1 - b2);
    //a1~a2の線とb1~b2の線の直交判定をする
}


bool isOrthogonal(Segment s1, Segment s2) {
    return equals(dot(s1.p2 - s1.p1, s2.p2 - s2.p1), 0.0);
    //線分s1とs2の直交判定をする
}

//平行判定
bool isParallel(Vector a, Vector b) {
    return equals(cross(a, b), 0.0);
    //ベクトルa,bの外積が0だったらa,bは平行
}

bool isParallel(Point a1, Point a2, Point b1, Point b2) {
    return isParallel(a1 - a2, b1 - b2);
    //a1~a2の線とb1~b2の線の平行判定をする
}

bool isParallel(Segment s1, Segment s2) {
    return equals(cross(s1.p2 - s1.p1, s2.p2 - s2.p1), 0.0);
    //線分s1とs2の平行判定をする
}


//射影
Point project(Segment s, Point p) {
    Vector base = s.p2 - s.p1;
    double r = dot(p - s.p1, base) / norm(base);
    return s.p1 + base * r;
}


//反射
Point reflect(Segment s, Point p) {
    return p + (project(s, p) - p) * 2.0;
}



//反時計回り
static const int COUNTER_CLOCKWISE = 1;
static const int CLOCKWISE = -1;
static const int ONLINE_BACK = 2;
static const int ONLINE_FRONT = -2;
static const int ON_SEGMENT = 0;

int ccw(Point p0, Point p1, Point p2) {
    Vector a = p1 - p0;
    Vector b = p2 - p0;
    if ( cross(a, b) > EPS ) return COUNTER_CLOCKWISE;
    if ( cross(a, b) < -EPS) return CLOCKWISE;
    if ( dot(a, b) < -EPS)   return ONLINE_BACK;
    if (a.norm() < b.norm()) return ONLINE_FRONT;

    return ON_SEGMENT;
}


//線分の交差判定
bool intersect(Point p1, Point p2, Point p3, Point p4) {
    return( ccw(p1, p2, p3) * ccw(p1, p2, p4) <= 0 &&
            ccw(p3, p4, p1) * ccw(p3, p4, p2) <= 0 );
}
bool intersect(Segment s1, Segment s2) {
    return intersect(s1.p1, s1.p2, s2.p1, s2.p2);
}


//距離
double getDistance(Point a, Point b) {  //点aと点bの距離
    return abs(a - b);
}
double getDistanceLP(Line l, Point p) { //直線lと点pの距離
    return abs(cross(l.p2 - l.p1, p - l.p1) / abs(l.p2 - l.p1));
}
double getDistanceSP(Segment s, Point p) {  //線分sと点pの距離
    if ( dot(s.p2 - s.p1, p - s.p1) < 0.0 ) return abs(p - s.p1);
    if ( dot(s.p1 - s.p2, p - s.p2) < 0.0 ) return abs(p - s.p2);
    return getDistanceLP(s, p);
}
double getDistance(Segment s1, Segment s2) {
    if ( intersect(s1, s2) ) return 0.0;
    return min(min(getDistanceSP(s1, s2.p1), getDistanceSP(s1, s2.p2)),
               min(getDistanceSP(s2, s1.p1), getDistanceSP(s2, s1.p2)));
}


//線分の交点
Point getCrossPoint(Segment s1, Segment s2) {
    Vector base = s2.p2 - s2.p1;
    double d1 = abs(cross(base, s1.p1 - s2.p1));
    double d2 = abs(cross(base, s1.p2 - s2.p1));
    double t = d1 / (d1 + d2);
    return s1.p1 + (s1.p2 - s1.p1) * t;
}


//円と直線の交点
pair<Point,Point> getCrossPoints(Circle c, Line l) {
    //assert(intersect(c, l));  ＜＝これ分からん
    Vector pr = project(l, c.c);
    Vector e = (l.p2 - l.p1) / abs(l.p2 - l.p1);
    double base = sqrt(c.r * c.r - norm(pr - c.c));
    return make_pair(pr + e * base, pr - e * base);
}


//円と円の交点
double arg(Vector p) {return atan2(p.x, p.y);}
Vector polar(double a, double r) {return Point(cos(r) * a, sin(r) * a);}

pair<Point, Point> getCrossPoints(Circle c1, Circle c2) {
    //assert(intersect(c1, c2));
    double d = abs(c1.c - c2.c);
    double a = acos((c1.r * c1.r + d * d - c2.r * c2.r) / (2 * c1.r * d));
    double t = arg(c2.c - c1.c);
    return make_pair(c1.c + polar(c1.r, t + a), c1.c + polar(c1.r, t - a));
}


int main(){
    cout<<"ok\n";
    return 0;
}
