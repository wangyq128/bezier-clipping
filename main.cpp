#include "myhead.h"
#include <GL/glut.h>
#include <time.h>
//输入控制点坐标，并在头文件中修改相应参数

//测试样例

//POW_P 3 POW_Q 4
//point con_p[] = { {0.0,0.0},{3.0,5.0},{11.0,5.0},{14.0,0.0} };
//point con_q[] = { {0,0},{6,-1},{10.0,8},{12.0,8.0},{14.0,0.0} };

//POW_P 6 POW_Q 6
//point con_p[] = { {-2,3},{0,0},{0,4},{7,5},{3,0},{0,2},{-2,3} };
//point con_q[] = { {2,-1.5},{-1,0.5},{3,0.5},{4,5.4262},{-1,3.5},{2,0.5},{2,-1.5} };

//POW_P 4 POW_Q 4
//point con_p[] = { {12.0,5.0},{10.0,2.0},{3.0,12.0},{11.0,12.0},{6.0,2.0} };
//point con_q[] = { {5.0,12.0},{2,10.0},{12.0,3.0},{12.0,11.0},{2.0,6.0} };

//POW_P 4 POW_Q 14
//point con_p[] = { {4,7},{14,7},{14,3},{4,3},{4,7} };
//point con_q[] = { {9,5},{7,10},{3,8},{3,0},{7,0},{11,12},{-3,10},{2,0},{8,2},{9,3},{10,9},{2,9},{2,0},{9,2},{9,5} };

point con_p[] = { {8,7},{15,7},{9,2},{2,7},{8,7} };
point con_q[] = { {12,5},{10,2},{0,12},{13,12},{6,2},{2,2},{0,9},{12,5} };

double psize, qsize;	//记录区间长度
double inter[MAXNUM]; //记录交点对应的参数

void bezier_clipping(point* p, point* q, double psize, double qsize, double left, int* time); //计算交点

//初始化背景颜色、投影矩阵
void InitScene()
{
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-2.0, 17.0, 0.0, 14.0);
}

//绘制bezier曲线
void DrawBezier(point* w, int pow_w, int step, double color[])
{
	glLineWidth(1.2);
	//线段起始点
	GLdouble v[2];
	GLdouble newV[2];

	//步长
	GLdouble deltat = 1.0 / step;
	GLdouble t = 0.0;

	//插值过程中的系数存储
	point c[MAXCON];
	for (int i = 0; i < MAXCON; i++)
		c[i].x = c[i].y = 0;

	//绘制控制曲线
	glColor3f(0.0, 0.0, 0.0);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i <= pow_w; i++)
		glVertex2f(w[i].x, w[i].y);
	glEnd();
	glFlush();

	

	//插值求点绘制曲线
	v[0] = w[0].x;v[1] = w[0].y;
	for (int l = 0; l < step; l++)
	{
		t += deltat;
		for (int i = 0; i <= pow_w; i++)
		{
			c[i].x = w[i].x;
			c[i].y = w[i].y;
		}
		for (int j = 1; j <= pow_w; j++)
		{
			for (int i = 0; i <= pow_w - j; i++)
			{
				c[i].x = (1 - t) * c[i].x + t * c[i + 1].x;
				c[i].y = (1 - t) * c[i].y + t * c[i + 1].y;
			}
		}
		newV[0] = c[0].x;
		newV[1] = c[0].y;
		glColor3f(color[0], color[1], color[2]);
		
		glBegin(GL_LINES);
		glVertex2dv(v);
		glVertex2dv(newV);
		glEnd();
		glFlush();
		v[0] = newV[0];v[1] = newV[1];
	}
}

//绘制曲线和交点
void DrawScene()
{
	double color1[3] = { 0.0,0.0,1.0 };
	DrawBezier(con_p, POW_P, 1000, color1);
	double color2[3] = { 0.0,0.5,0.0 };
	DrawBezier(con_q, POW_Q, 1000, color2);

	//画交点
	point c[POW_P + 1];
	point interpoint[MAXNUM];
	for (int i = 0; i <= POW_P; i++)
		c[i].x = c[i].y = 0;
	int num = 0;//记录交点数量

	//inter[3] = 0.373045;
	//inter[4] = 0.12;
	printf("交点坐标为:\n");
	//求交点
	for (int l = 0; l < MAXNUM; l++)
	{
		if (inter[l] != FAIL)
		{
			for (int i = 0; i <= POW_P; i++)
			{
				c[i].x = con_p[i].x;
				c[i].y = con_p[i].y;
			}
			for (int j = 1; j <= POW_P; j++)
			{
				for (int i = 0; i <= POW_P - j; i++)
				{
					c[i].x = (1 - inter[l]) * c[i].x + inter[l] * c[i + 1].x;
					c[i].y = (1 - inter[l]) * c[i].y + inter[l] * c[i + 1].y;
				}
			}
			interpoint[num].x = c[0].x;
			interpoint[num].y = c[0].y;
			printf("(%f,%f)\n", interpoint[num].x, interpoint[num].y);
			num += 1;
		}

	}
	printf("\n");

	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_POINT_SMOOTH);
	glPointSize(5.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < num; i++)
		glVertex2f(interpoint[i].x, interpoint[i].y);
	//glVertex2f(6.682266, 1.482840);
	glEnd();
	glDisable(GL_POINT_SMOOTH);
	glFlush();
}


int main(int argc, char** argv)
{
	printf("平面Bezier曲线求交程序\n");
	printf("P的控制点为:");
	for (int i = 0; i < POW_P; i++)
		printf("(%3f,%3f)", con_p[i].x, con_p[i].y);
	printf("\n");
	printf("Q的控制点为:");
	for (int i = 0; i < POW_Q; i++)
		printf("(%3f,%3f)", con_q[i].x, con_q[i].y);
	printf("\n");

	printf("......求交......\n");

	//初始化参数范围
	psize = qsize = 1.0;

	for (int i = 0; i < MAXNUM; i++)
		inter[i] = -1;

	//调用函数求交点对应的参数
	int* time = new int();//标识切割段数
	*time = 0;

	double start, end;
	double clipping_time;
	start = clock();
	bezier_clipping(con_p, con_q, psize, qsize, 0, time);
	end = clock();
	clipping_time = (end - start) * 1000 / CLOCKS_PER_SEC;
	printf("时间:%f ms\n", clipping_time);
	printf("交点对应的参数:");
	for (int i = 0; i < MAXNUM; i++)
		if (inter[i] != FAIL)
			printf("%f  ", inter[i]);
	printf("\n");


	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
	glutInitWindowSize(600, 600);
	glutInitWindowPosition(500, 50);
	glutCreateWindow("bezier clipping");

	InitScene();
	glutDisplayFunc(DrawScene);

	glutMainLoop();

}

