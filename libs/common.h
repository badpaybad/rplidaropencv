
#ifndef sys__lockGlobal
#define sys__lockGlobal
static std::mutex __lockGlobal ;
#endif

#ifndef _pi31416
#define _pi31416 3.1416f;
#endif

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]));
#endif

#ifndef sys__RplidarPoint
#define sys__RplidarPoint

struct RplidarPoint
{
    int _x;
    int _y;
    int _Ox;
    int _Oy;
    float _angle;
    float _distance;
    float _mapRatio;
    int _scanDiameter;

public:
    RplidarPoint() {}
    RplidarPoint(int x, int y, int Ox, int Oy, float mapRatio, int scanDiameter, float angle, float distance)
    {
        _x = x;
        _y = y;
        _Ox = Ox;
        _Oy = Oy;
        _angle = angle;
        _distance = distance;
        _mapRatio = mapRatio;
        _scanDiameter = scanDiameter;
    }
    RplidarPoint operator=(RplidarPoint input)
    {
        _x = input._x;
        _y = input._y;
        _Ox = input._Ox;
        _Oy = input._Oy;
        _angle = input._angle;
        _distance = input._distance;
        _mapRatio = input._mapRatio;
        _scanDiameter = input._scanDiameter;

        return input;
    }
} ;

const int _maxPoints = 8192;

#endif