
#include <queue>
#include <vector>
#include <iostream>
#include <windows.h>
#include <math.h>
#include <ctime>
#define START (5)       // 定义起点的标记数
#define END (9)         // 定义终点的标记数
#define SEAL (2)        // 定义墙壁的标记数
#define FILL (1)		// 定义填充路径的标记
#define mazeSavePath "D:\\map.txt"
int row, col;
int sx, sy, ex, ey;
int** map;

void menu();
void mazeSave();
void mazeLoad();
void gamestartSingle();


using namespace std;

int vectors[8][2] = { {0, 1},{-1, 0},{0, -1},{1, 0},{1, 1},{-1, 1},{-1, -1},{1, -1} };



// 
enum FLAG { CLOSE, UNVISTED, OPEN };

//
typedef struct mind {
    int _x;		//	坐标x
    int _y;		//	坐标y
    float _G;		//	实际行进的距离
    float _H;		//	最佳路径的估计代价
    float _F;		//	评估函数
    FLAG flag;	//	结点的状态标识量
    mind* pre;	//	前驱结点
}*m_ptr;


class AStar {
    int** _map;		//	指向整数标记的动态二维数组
    mind** _minds;		//	指向结点的动态二维数组
    int _start_x, _start_y;		//	起点坐标
    int _end_x, _end_y;			//	终点坐标
    int width;			//	迷宫宽
    int height;			//	迷宫高

    //	仿函数结构体，用于生成priority_queue优先队列
    struct cmp {
        bool operator()(m_ptr m1, m_ptr m2) {
            return m1->_F > m2->_F;
        }
    };

    // 优先队列，push入元素时实现自动排序
    priority_queue<m_ptr, vector<m_ptr>, cmp> _open;

    // 评估代价的取得函数
    float get_H(int x, int y) {
        float dx = abs(x - _end_x);
        float dy = abs(y - _end_y);
        return sqrt(dx * dx + dy * dy);
    }

    // 测试下标是否符合规范
    bool bound(int x, int y) {
        if (x < 0 || y < 0 || x >= height || y >= width) return false;
        return true;
    }

    // 动态分配内存
    void allocate(int width, int height) {
        _map = new int* [height];
        _minds = new m_ptr[height];
        int i;
        for (i = 0; i < height; i++) {
            _map[i] = new int[width];
            _minds[i] = new mind[width];
        }
    }

    // 进行初始化，然后使用A*算法计算出路径
public: int** initial() {
        int i, j;
        for (i = 0; i < height; i++) {
            for (j = 0; j < width; j++) {
                _minds[i][j].pre = NULL;
                _minds[i][j]._F = 0;
                _minds[i][j]._H = 0;
                _minds[i][j]._G = 0;

                // 写入坐标
                _minds[i][j]._x = i;
                _minds[i][j]._y = j;


                switch (_map[i][j]) {
                    // 遇到墙壁，结点标识设置为CLOSE
                case SEAL:
                    _minds[i][j].flag = CLOSE; break;
                    // 找到起点，设置坐标
                case START:
                    _start_x = i, _start_y = j; break;
                    // 找到终点，设置坐标
                case END:
                    _end_x = i, _end_y = j;
                    // 对于起点，终点，通路，统一设置相应结点的标识为UNVISTED
                default:
                    _minds[i][j].flag = UNVISTED; break;
                }
            }
        }

        calculate();
        return _map;
    }

    // draw函数辅助步骤，向_map数组中写入路径标记，使用FILL的预定义
    void fill(m_ptr p) {
        if (p) {
            fill(p->pre);
            if (_map[p->_x][p->_y] != START && _map[p->_x][p->_y] != END)
                _map[p->_x][p->_y] = 1;
        }
    }

    // Astar 算法核心步骤
    void calculate() {
        m_ptr p = _minds[_start_x] + _start_y;
        p->flag = OPEN;
        p->_H = get_H(p->_x, p->_y);
        p->_G = 0.0;
        p->_F = p->_H + p->_G;
        _open.push(p);


        while (!_open.empty()) {
            p = _open.top();
            _open.pop();

            p->flag = CLOSE;
            int x = p->_x;
            int y = p->_y;

            if (x == _end_x && y == _end_y) return;

            for (int i = 0; i < 8; i++) {
                bool direct = i < 4;

                int xx = x + vectors[i][0];
                int yy = y + vectors[i][1];
                m_ptr n = _minds[xx] + yy;

                if (!bound(xx, yy) || n->flag == CLOSE || _map[xx][yy] == SEAL) continue;

                if (n->flag == UNVISTED) {
                    n->flag = OPEN;
                    n->pre = p;
                    n->_G = p->_G + (direct ? 1.0 : sqrt(2));
                    n->_H = get_H(xx, yy);
                    n->_F = n->_G + n->_H;
                    _open.push(n);
                }
                else {
                    bool case1 = (p->_G + 1 < n->_G) && i < 4;
                    bool case2 = (p->_G + sqrt(2) < n->_G) && i >= 4;

                    if (case1 || case2) {
                        n->pre = p;
                        n->_G = p->_G + (direct ? 1.0 : sqrt(2));
                        n->_F = n->_G + n->_H;
                    }
                }
            }
        }
    }
public:
    AStar(int w, int h) {

        
        _start_x = _start_y = _end_x = _end_y = 0;
        width = w;
        height = h;
        allocate(width, height);
        int i, j;
        for (i = 0; i < height; i++) {
            for (j = 0; j < width; j++) {
                _map[i][j] = map[i][j]; 
            }
        }

        initial();
    }

    ~AStar() {
        int i;
        for (i = 0; i < height; i++) {
            delete[] _map[i];
            delete[] _minds[i];
        }
        delete[] _map;
        delete[] _minds;
    }


    // 绘制迷宫与路径的函数
    public: int** draw(bool printornot) {
        fill(_minds[_end_x] + _end_y);
        int i, j;
        if(printornot)
        { 
            for (i = 0; i < height; i++) {
                for (j = 0; j < width; j++) {
                    switch (_map[i][j]) {
                        // 红色显示起点，和终点
                    case START: cout << " S";  break;
                    case END: cout << " E";  break;
                        // 绿色显示路径
                    case FILL: cout << " o"; break;
                        // 墙壁显示为白块
                    case SEAL:cout << " X"; break;
                    default: cout << "  "; break;
                    }
                }
                cout << endl;
            }
        }

        return _map;
    }



};

void creatMap(int row, int col,int sx, int sy)
{
    map = new int* [row];
    for (int i = 0; i < row; i++)
    {
        map[i] = new int[col];

    }
    int maxway = row * col;		//最大通路
    int x, y;

    for (x = 0; x < row; x++)
        for (y = 0; y < col; y++)
            map[x][y] = 2;			//先填充迷宫

    srand((unsigned)time(NULL));	//随机函数种子，以时间为参数
    for (int i = 0; i < maxway; i++)		//随机构建迷宫通路
    {
        x = rand() % (row - 2) + 1;
        y = rand() % (col - 2) + 1;
        map[x][y] = 0;
    }

    map[1][1] = 0;
    map[1][col - 2] = 0;
    map[row - 2][1] = 0;
    map[row - 2][col - 2] = 0; //四角必须有同路

    map[sx][sy] = 5;
    map[row/2][col/2] = END;


}

void gamestartSingle()
{
    cout << "\n      Please input Rows and Colms with Enter :" << endl;
    cin >> row >> col;

    if (row < 5 && col < 5)
    {
        std::cout << "\n      Rows & Colms Error! Please input more than 5 to make sure center space! Retry!" << endl;
        cin >> row >> col;
    }
    cout << "\n      Please input StartCoords(x,y) with Enter :"<<endl;
    cin >> sx >> sy;
    if (sx == 0 || sy == 0 || sx == row - 1 || sy == col - 1)
    {
        creatMap(row, col, sx, sy);

        AStar as(row, col);
        as.draw(true);
        menu();
    }
    else
    {
        cout<< "\n      StartCoords(x,y) Error! Please Retry!" << endl;
        gamestartSingle();
    }


}


void gamestartDouble()
{
    cout << "\n      Please input Rows and Colms with Enter :" << endl;
    cin >> row >> col;
    if (row < 5 && col < 5)
    {
        cout << "\n      Rows & Colms Error! Please input more than 5 to make sure center space! Retry!" << endl;
        cin >> row >> col;
    }
    // 人数

    cout << "\n      Please input the number of Players!" << endl;
    int count = 0;
    cin >> count;
    int* x_list = new int[count];
    int* y_list = new int[count];


    for (int i = 0; i < count; i++)
    {
        cout << "\n      Please input No."<< (i+1) << " StartCoords(x,y) with Enter :" << endl;
        cin >> sx >> sy;
        x_list[i] = sx;
        y_list[i] = sy;
    }

    creatMap(row, col, x_list[0], y_list[0]);
    int** record_map = new int*[row];
    for (int i = 0; i < row; i++)
    {
        record_map[i] = new int[col];
        for (int j = 0; j < col; j++)
        {
            record_map[i][j] = map[i][j];// 没路的 map

        }
    }
    
    int** catchMap; 
    int i = 0;  
    do
    {
            AStar as(row, col);
            catchMap = as.draw(false);
          
            for (int i = 0; i < row; i++)
            {
                for (int j = 0; j < col; j++)
                {
                    if (catchMap[i][j] == 1)
                    {
                        record_map[i][j] = 1;
                    }
                }
                cout << endl;
            }

            map[x_list[i]][y_list[i]] = 0;
            if (i < count-1)
            {
                map[x_list[i + 1]][y_list[i + 1]] = 5;
            }
            i++;
           

    } while (i < count);

    //打印 map
    for (int i = 0; i<count;i++)
    {
        record_map[x_list[i]][y_list[i]] = 5;
    }
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            switch (record_map[i][j]) {
                // 红色显示起点，和终点
            case START: cout << " S";  break;
            case END: cout << " E";  break;
                // 绿色显示路径
            case FILL: cout << " o"; break;
                // 墙壁显示为白块
            case SEAL:cout << " X"; break;
            default: cout << "  "; break;
            }
        }
        cout << endl;
    }

    menu();

}

void mazeSave()
{
    FILE* fp;
    errno_t err;
    err = fopen_s(&fp, mazeSavePath, "w");
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            fprintf(fp, "%d ", map[i][j]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    cout << "\n               Maze Save Successful! Path: D:\\maze.txt " << endl;
    menu();
}

void mazeLoad()
{

    int i, j;

    FILE* fp;
    errno_t err;
    //err = fopen_s(&fp, readDataPath, "w");
    err = fopen_s(&fp, mazeSavePath, "r"); //打开文件  
    if (fp == NULL)
    {
        printf("File Open Error!");
        return;
    }
    for (i = 0; i < row; i++)
    {
        for (j = 0; j < col; j++)
        {
            fscanf_s(fp, "%d", &map[i][j]); /*每次读取一个数，fscanf函数遇到空格或者换行结束*/
        }
        fscanf_s(fp, "\n");
    }
    fclose(fp);
    AStar as(row, col);
    as.draw(true);
    menu();


}

void menu()
{
    cout << "\t*******************************************************************" << endl;
    cout << "\t*                                                                 *" << endl;
    cout << "\t*       1.Start Game with Single Person to see Shortest Path      *" << endl;
    cout << "\t*                                                                 *" << endl;
    cout << "\t*    2.Start Game with More than 2 person to find Shortest Path   *" << endl;
    cout << "\t*                                                                 *" << endl;
    cout << "\t*               3.          Exit                                  *" << endl;
    cout << "\t*                                                                 *" << endl;
    cout << "\t*               4.         Save Maze                              *" << endl;
    cout << "\t*                                                                 *" << endl;
    cout << "\t*               5.         Load Maze                              *" << endl;
    cout << "\t*                                                                 *" << endl;
    cout << "\t*******************************************************************" << endl;
    

    switch (getchar()) {
    case '1':
        gamestartSingle(); break;
    case '2':
        gamestartDouble();
        break;
    case '3':
        exit(1); break;
    case '4':
        mazeSave();
        break;
    case '5':
        mazeLoad();
        break;
    default:
        cout << "\n\n              Error input! Press Enter to Back!" << endl;
        getchar();
        menu();
    }
    getchar();
}



// 主函数
int main() {
  
    menu();
    return 0;
}


