#include "myhead.h"

extern double inter[MAXNUM];

//根据参数范围求出新的控制点
void control_point(point* w, point* ww, int pow_w, double a, double b)
{

	point c[MAXCON];
	for (int i = 0; i < MAXCON; i++)
		c[i].x = c[i].y = 0;

	//算0到b的控制点
	for (int i = 0; i <= pow_w; i++)
	{
		c[i].x = w[i].x;
		c[i].y = w[i].y;
	}
	for (int j = 1; j <= pow_w; j++)
	{
		for (int i = pow_w; i >= j; i--)
		{
			c[i].x = (1 - b) * c[i - 1].x + b * c[i].x;
			c[i].y = (1 - b) * c[i - 1].y + b * c[i].y;
		}
	}

	//算a到b的控制点
	double u = a / b;

	for (int j = 1; j <= pow_w; j++)
	{
		for (int i = 0; i <= pow_w - j; i++)
		{
			c[i].x = (1 - u) * c[i].x + u * c[i + 1].x;
			c[i].y = (1 - u) * c[i].y + u * c[i + 1].y;
		}
	}

	//赋值
	for (int l = 0; l <= pow_w; l++)
	{
		ww[l].x = c[l].x;
		ww[l].y = c[l].y;
	}

}

//求出数组中的最大值
double maxdist(double x[], int pow)
{
	double d_max = -10000.0;
	for (int j = 0; j <= pow; j++)
		if (x[j] > d_max) d_max = x[j];
	return d_max;
}

//求出数组中的最小值
double mindist(double x[], int pow)
{
	double d_min = 10000.0;
	for (int j = 0; j <= pow; j++)
		if (x[j] < d_min) d_min = x[j];
	return d_min;
}

//求距离曲线的凸包并确定切割上下限
void convex_bound(double dist_w[], int pow_w, double d_max, double d_min, double* w1, double* w2)
{
	//判断是否有交点
	double dd_max = maxdist(dist_w, pow_w);
	double dd_min = mindist(dist_w, pow_w);
	if (dd_max < d_min || dd_min > d_max)
	{
		*w1 = FAIL;
		return;
	}


	//计算凸包顶点
	int convex[MAXCON];//储存凸包顶点对应的标号
	for (int i = 0; i < MAXCON; i++) convex[i] = -1; //初始化
	convex[0] = 0;//将最左边的点加到凸包中

	//加入第一个点
	double bound = -100000; double temp;
	for (int i = 1; i <= pow_w; i++)
	{
		if ((temp = (dist_w[i] - dist_w[0]) / i) > bound)
		{
			bound = temp;
			convex[1] = i;
		}
	}

	//加入后续的点
	int index = 1;
	bound = -100000;
	while (convex[index] != 0)
	{
		index += 1;
		int j = index - 1;

		//计算上一步向量
		point last = { 0,0 };
		point now = { 0,0 };
		last.x = convex[j] - convex[j - 1];
		last.y = dist_w[convex[j]] - dist_w[convex[j - 1]];
		double norm_last = sqrt(last.x * last.x + last.y * last.y);
		double norm_now;
		bound = -1.1;
		for (int i = 0; i <= pow_w; i++)
		{
			if (i != convex[j])
			{
				//计算余弦值
				now.x = i - convex[j];
				now.y = dist_w[i] - dist_w[convex[j]];
				norm_now = sqrt(now.x * now.x + now.y * now.y);
				temp = (last.x * now.x + last.y * now.y) / (norm_now * norm_last);
				if (temp > bound)
				{
					bound = temp;
					convex[index] = i;
				}
			}

		}
	}
	/*for(int i = 0; i <= index; i++)
		printf("convex[%d]:%d\t", i, convex[i]);
	printf("\n");*/
	//凸包有index + 1个顶点

	//计算d_min和凸包的交点
	double in[] = { 0,0,0,0,0 };
	int l = 0;
	for (int i = 0; i < index; i++)
	{
		if ((d_min >= dist_w[convex[i]] && d_min <= dist_w[convex[i + 1]]) || (d_min >= dist_w[convex[i + 1]] && d_min <= dist_w[convex[i]]))
		{
			if (dist_w[convex[i + 1]] == dist_w[convex[i]])
			{
				in[0] = convex[i];
				in[1] = convex[i + 1];
				break;
			}
			in[l] = (convex[i + 1] - convex[i]) * (d_min - dist_w[convex[i]]) / (pow_w * (dist_w[convex[i + 1]] - dist_w[convex[i]])) + convex[i] / double(pow_w);
			if (in[l] != in[l - 1])
				l++;
		}
	}
	if (l == 0) //说明没有交点
		in[0] = in[1] = (dist_w[pow_w] - d_min >= dist_w[0] - d_min) ? 0 : 1;
	l = 2;

	//计算d_max和凸包的交点
	for (int i = 0; i < index; i++)
	{
		if ((d_max >= dist_w[convex[i]] && d_max <= dist_w[convex[i + 1]]) || (d_max >= dist_w[convex[i + 1]] && d_max <= dist_w[convex[i]]))
		{
			if (dist_w[convex[i + 1]] == dist_w[convex[i]])
			{
				in[2] = convex[i];
				in[3] = convex[i + 1];
				break;
			}
			in[l] = (convex[i + 1] - convex[i]) * (d_max - dist_w[convex[i]]) / (pow_w * (dist_w[convex[i + 1]] - dist_w[convex[i]])) + convex[i] / double(pow_w);
			if (in[l] != in[l - 1])
				l++;
		}
	}

	if (l == 2) //说明没有交点
		in[2] = in[3] = (d_max - dist_w[pow_w] > d_max - dist_w[0]) ? 0 : 1;

	//赋值
	if (dist_w[0] >= d_min && dist_w[0] <= d_max)
	{
		if (dist_w[pow_w] >= d_min && dist_w[pow_w] <= d_max)
		{
			*w1 = 0; *w2 = 1;
		}
		else
		{
			*w1 = 0;
			*w2 = maxdist(in, 3);
		}
	}
	else if (dist_w[0] < d_min || dist_w[0] > d_max)
	{
		if (dist_w[pow_w] >= d_min && dist_w[pow_w] <= d_max)
		{
			*w1 = mindist(in, 3);
			*w2 = 1;
		}
		else
		{
			if (in[0] == in[1] && in[2] == in[3])
			{
				*w1 = (in[0] < in[2]) ? in[0] : in[2];
				*w2 = (in[0] < in[2]) ? in[2] : in[0];
			}
			else if (in[0] != in[1] && in[2] == in[3])
			{
				double tem = (fabs(in[0] - in[2]) > fabs(in[1] - in[2])) ? in[0] : in[1];
				*w1 = (tem < in[2]) ? tem : in[2];
				*w2 = (tem < in[2]) ? in[2] : tem;
			}
			else if (in[0] == in[1] && in[2] != in[3])
			{
				double tem = (fabs(in[0] - in[2]) > fabs(in[0] - in[3])) ? in[2] : in[3];
				*w1 = (tem < in[0]) ? tem : in[0];
				*w2 = (tem < in[0]) ? in[0] : tem;
			}
			else
			{
				*w1 = mindist(in, 3);
				*w2 = maxdist(in, 3);
			}
		}
	}
	return;

}

void bezier_clipping(point* con_p, point* con_q, double psize, double qsize, double left, int* time)
{
	//两条曲线在两端相交
	/*
	if (((con_p[0].x == con_q[POW_P].x && con_p[0].y == con_q[POW_P].y) || (con_q[0].x == con_p[POW_P].x && con_q[0].y == con_p[POW_P].y)) && *time == 0)
	{
		if (con_p[0].x == con_q[POW_P].x && con_p[0].y == con_q[POW_P].y)
		{
			inter[*time] = 0;
			*time += 1;
		}
		if (con_q[0].x == con_p[POW_P].x && con_q[0].y == con_p[POW_P].y)
		{
			inter[*time] = 1;
			*time += 1;
		}
	}
	*/

	if (psize < 0.001) {
		inter[*time] = 0;
		return;
	}

	//首尾相接的情况
	if (con_p[0].x == con_p[POW_P].x && con_p[0].y == con_p[POW_P].y)
	{
		point con_split1[POW_P + 1];
		point con_split2[POW_P + 1];

		//把p分成两段
		control_point(con_p, con_split1, POW_P, 0, 0.5);
		control_point(con_p, con_split2, POW_P, 0.5, 1);
		psize = psize * 0.5;
		bezier_clipping(con_split1, con_q, psize, qsize, left, time);//处理左段
		if (inter[*time] != FAIL) {
			inter[*time] = inter[*time] * psize + left;
			//printf("inter[%d]:%f\n", *time, inter[*time]);
			*time += 1;
		}
		//printf("\n");

		bezier_clipping(con_split2, con_q, psize, qsize, left + psize, time);//处理右段
		if (inter[*time] != FAIL) {
			inter[*time] = inter[*time] * psize + left + psize;
			//printf("inter[%d]:%f\n", *time, inter[*time]);
			*time += 1;
		}
		//printf("\n");
		return;
	}
	else if (con_q[0].x == con_q[POW_P].x && con_q[0].y == con_q[POW_P].y)
	{
		point con_split1[POW_Q + 1];
		point con_split2[POW_Q + 1];

		qsize = qsize * 0.5; //要对clip前的q做处理
		//printf("psize:%f \t qsize:%f \t left:%f\n", psize, qsize, left);
		//printf("\n\n");

		//把q分成两段
		control_point(con_q, con_split1, POW_Q, 0, 0.5);
		control_point(con_q, con_split2, POW_Q, 0.5, 1);
		bezier_clipping(con_p, con_split1, psize, qsize, left, time);
		if (inter[*time] != FAIL) {
			inter[*time] = inter[*time] * psize + left;
			//printf("inter[%d]:%f\n", *time, inter[*time]);
			*time += 1;
		}
		//printf("\n");
		bezier_clipping(con_p, con_split2, psize, qsize, left, time);
		if (inter[*time] != FAIL) {
			inter[*time] = inter[*time] * psize + left;
			//printf("inter[%d]:%f\n", *time, inter[*time]);
			*time += 1;
		}
		return;
	}
	else
	{
		/*for(int i = 0; i <= POW_P; i++)
			printf("p%d (%f,%f)\t",i, con_p[i].x, con_p[i].y);
		printf("\n");
		for (int i = 0; i <= POW_Q; i++)
			printf("q%d (%f,%f)\t", i, con_q[i].x, con_q[i].y);
		printf("\n");*/

		double p1, p2, q1, q2;	//记录参数范围
		p1 = q1 = 0;
		p2 = q2 = 1;
		double dist_p[POW_P + 1];		//记录p的控制点到参考线的距离
		double dist_q[POW_Q + 1];		//记录q的控制点到参考线的距离
		double d_max, d_min;			//记录距离上下限
		double coef[4] = { 0,0,0,0 };		//记录参考线的系数

		//切割q
		//计算参考线的系数
		coef[0] = con_p[0].y - con_p[POW_P].y;
		coef[1] = con_p[POW_P].x - con_p[0].x;
		coef[2] = con_p[0].x * con_p[POW_P].y - con_p[POW_P].x * con_p[0].y;
		coef[3] = sqrt(coef[0] * coef[0] + coef[1] * coef[1]);
		if (coef[3] != 0)
		{
			coef[0] /= coef[3];
			coef[1] /= coef[3];
			coef[2] /= coef[3];
		}

		/*printf("coef[0]:%f \t coef[1]:%f \t coef[2]: %f \n", coef[0], coef[1], coef[2]);*/

		//计算距离
		dist_p[0] = dist_p[POW_P] = 0;
		for (int i = 1; i < POW_P; i++)
			dist_p[i] = coef[0] * con_p[i].x + coef[1] * con_p[i].y + coef[2];

		for (int i = 0; i <= POW_Q; i++)
			dist_q[i] = coef[0] * con_q[i].x + coef[1] * con_q[i].y + coef[2];

		d_max = maxdist(dist_p, POW_P);
		d_min = mindist(dist_p, POW_P);
		/*printf("d_max:%f\t d_min:%f\n", d_max, d_min);*/

		//缩小参数范围
		convex_bound(dist_q, POW_Q, d_max, d_min, &q1, &q2);
		/*printf("q1:%f\t q2:%f\n", q1, q2);*/

		//无交点情形
		if (q1 == FAIL)
		{
			inter[*time] = FAIL;
			return;
		}
		//端点相交情形
		if (q1 == q2)
		{
			if (q1 == 0)
			{
				if (con_q[0].x == con_p[0].x && con_q[0].y == con_p[0].y)
					inter[*time] = 0;
				else if (con_q[0].x == con_p[POW_P].x && con_q[0].y == con_p[POW_Q].y)
					inter[*time] = 1;
				else
					inter[*time] = FAIL;
			}
			else if (q1 == 1)
			{
				if (con_q[POW_Q].x == con_p[0].x && con_q[POW_Q].y == con_p[0].y)
					inter[*time] = 0;
				else if (con_q[POW_Q].x == con_p[POW_P].x && con_q[POW_Q].y == con_p[POW_Q].y)
					inter[*time] = 1;
				else
					inter[*time] = FAIL;
			}
			else
				inter[*time] = FAIL;

			return;
		}

		//计算切割后的控制点
		point con_qq[POW_Q + 1];
		control_point(con_q, con_qq, POW_Q, q1, q2);
		/*for (int i = 0; i <= POW_Q; i++)
			printf("qq%d (%f,%f)\t", i, con_qq[i].x, con_qq[i].y);
		printf("\n");*/

		//切割p
		//计算参考线的系数
		coef[0] = con_qq[0].y - con_qq[POW_Q].y;
		coef[1] = con_qq[POW_Q].x - con_qq[0].x;
		coef[2] = con_qq[0].x * con_qq[POW_Q].y - con_qq[POW_Q].x * con_qq[0].y;
		coef[3] = sqrt(coef[0] * coef[0] + coef[1] * coef[1]);
		if (coef[3] != 0)
		{
			coef[0] /= coef[3];
			coef[1] /= coef[3];
			coef[2] /= coef[3];
		}

		//计算距离
		dist_q[0] = dist_q[POW_Q] = 0;
		for (int i = 1; i < POW_Q; i++)
			dist_q[i] = coef[0] * con_qq[i].x + coef[1] * con_qq[i].y + coef[2];

		for (int i = 0; i <= POW_P; i++)
			dist_p[i] = coef[0] * con_p[i].x + coef[1] * con_p[i].y + coef[2];
		d_max = maxdist(dist_q, POW_Q);
		d_min = mindist(dist_q, POW_Q);
		/*printf("d_max:%f\t d_min:%f\n", d_max, d_min);*/

		//缩小参数范围
		convex_bound(dist_p, POW_P, d_max, d_min, &p1, &p2);
		/*printf("p1:%f\t p2:%f\n", p1, p2);*/

		//无交点情形
		if (p1 == FAIL)
		{
			inter[*time] = FAIL;
			return;
		}
		//端点相交情形
		if (p1 == p2)
		{
			if (p1 == 0)
			{
				if (con_q[0].x == con_p[0].x && con_q[0].y == con_p[0].y || con_q[POW_Q].x == con_p[0].x && con_q[POW_Q].y == con_p[0].y)
					inter[*time] = 0;
				else
					inter[*time] = FAIL;
			}
			else if (p1 == 1)
			{
				if (con_q[0].x == con_p[POW_P].x && con_q[0].y == con_p[POW_P].y || con_q[POW_Q].x == con_p[POW_P].x && con_q[POW_Q].y == con_p[POW_P].y)
					inter[*time] = 1;
				else
					inter[*time] = FAIL;
			}
			else
				inter[*time] = FAIL;

			return;
		}

		//计算切割后的控制点
		point con_pp[POW_P + 1];
		control_point(con_p, con_pp, POW_P, p1, p2);

		/*for (int i = 0; i <= POW_P; i++)
			printf("pp%d (%f,%f)\t", i, con_pp[i].x, con_pp[i].y);
		printf("\n");*/


		//判断是否有多个交点
		if (p2 - p1 > 0.7 || q2 - q1 > 0.7)
		{
			if (((p2 - p1) * psize) >= ((q2 - q1) * qsize))//切割p
			{
				point con_split1[POW_P + 1];
				point con_split2[POW_P + 1];
				psize = psize * 0.5;

				//printf("psize:%f \t qsize:%f \t left:%f\n", psize, qsize, left);
				//printf("\n\n");

				//把p分成两段
				control_point(con_p, con_split1, POW_P, 0, 0.5);
				control_point(con_p, con_split2, POW_P, 0.5, 1);
				bezier_clipping(con_split1, con_q, psize, qsize, left, time);//处理左段
				if (inter[*time] != FAIL) {
					inter[*time] = inter[*time] * psize + left;
					//printf("inter[%d]:%f\n", *time, inter[*time]);
					*time += 1;
				}
				//printf("\n");
				bezier_clipping(con_split2, con_q, psize, qsize, left + psize, time);//处理右段
				if (inter[*time] != FAIL) {
					inter[*time] = inter[*time] * psize + left + psize;
					//printf("inter[%d]:%f\n", *time, inter[*time]);
					*time += 1;
				}
				//printf("\n");
				return;
			}
			else if (((p2 - p1) * psize) < ((q2 - q1) * qsize))
			{
				point con_split1[POW_Q + 1];
				point con_split2[POW_Q + 1];

				qsize = qsize * 0.5; //要对clip前的q做处理
				//printf("psize:%f \t qsize:%f \t left:%f\n", psize, qsize, left);
				//printf("\n\n");

				//把q分成两段
				control_point(con_q, con_split1, POW_Q, 0, 0.5);
				control_point(con_q, con_split2, POW_Q, 0.5, 1);
				bezier_clipping(con_p, con_split1, psize, qsize, left, time);
				if (inter[*time] != FAIL) {
					inter[*time] = inter[*time] * psize + left;
					//printf("inter[%d]:%f\n", *time, inter[*time]);
					*time += 1;
				}
				//printf("\n");
				bezier_clipping(con_p, con_split2, psize, qsize, left, time);
				if (inter[*time] != FAIL) {
					inter[*time] = inter[*time] * psize + left;
					//printf("inter[%d]:%f\n", *time, inter[*time]);
					*time += 1;
				}


				//printf("\n");
				return;
			}

		}

		//至多只有一个交点
		//切割后的实际长度和左值
		left = left + p1 * psize;
		psize = (p2 - p1) * psize;
		qsize = (q2 - q1) * qsize;
		//printf("psize:%f \t qsize:%f \t left:%f\n", psize, qsize,left);
		//printf("\n\n");

		//迭代
		bezier_clipping(con_pp, con_qq, psize, qsize, left, time);

		if (inter[*time] != FAIL)
			inter[*time] = inter[*time] * (p2 - p1) + p1;
		//printf("inter[%d]:%f\n", *time, inter[*time]);

		return;
	}


}

