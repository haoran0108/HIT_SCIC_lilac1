/*
 * Image.c
 *
 *  Created on: 2021��10��28��
 *      Author: windows
 */
#include "image.h"
//aaaaa
int f[10 * CAMERA_H];//������ͨ����ͨ��
int zebraCount = 0, zFlag = 0;

uint8_t* fullBuffer = &mt9v034_image[0][0];


//ÿ������������
typedef struct {
    uint8_t   left;//��߽�
    uint8_t   right;//�ұ߽�
    int   connect_num;//��ͨ���
}range;

//ÿ�е����а�����
typedef struct {
    uint8_t   num;//ÿ�а�������
    range   area[white_num_MAX];//���и���������
}all_range;

//����������ÿ������������
typedef struct {
    uint8_t   left;//��߽�
    uint8_t   right;//�ұ߽�
    uint8_t   width;//���
}road_range;

//ÿ������������ÿ��������
typedef struct {
    uint8_t   white_num;
    road_range   connected[white_num_MAX];
}road;

all_range white_range[CAMERA_H];//���а�����
road my_road[CAMERA_H];//����
uint8_t IMG[CAMERA_H][CAMERA_W];//��ֵ����ͼ������
uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//���������ұ߽�
uint8_t mid_line[CAMERA_H];
int all_connect_num = 0;//���а�������
uint8_t top_road;//������ߴ���������
uint8_t threshold = 160;//��ֵ

////////////////////////////////////////////
//���ܣ���ֵ��
//���룺�Ҷ�ͼƬ
//�������ֵ��ͼƬ
//��ע��
///////////////////////////////////////////
void THRE()
{
    uint8_t* map;
    uint8_t* my_map;
    int count1 = 0;
    int count2 = 0;
    for (int i = 40; i < 50; i++)
    {
        for (int j = 148; j <= 155; j++)
        {
            count1 = 0;
            for (int y = i - 2; y <= i + 2; y++)
            {
                for (int x = j - 2; x <= j + 2; x++)
                {
                    map = fullBuffer + 188 * y + x - 1;
                    count1 = count1 + (*map);
                }
            }
            map = fullBuffer + 188 * i + j - 1;
            (*map) = count1 / 25;
        }
    }
    for (int i = 43; i < 62; i++)
    {
        for (int j = 4; j <= 24; j++)
        {
            count2 = 0;
            for (int y = i - 2; y <= i + 2; y++)
            {
                for (int x = j - 2; x <= j + 2; x++)
                {
                    map = fullBuffer + 188 * y + x - 1;
                    count2 = count2 + (*map);
                }
            }
            map = fullBuffer + 188 * i + j - 1;
            (*map) = count2 / 25;
        }
    }

    map = fullBuffer;
    for (int i = 0; i < 120; i++)
    {
        my_map = &IMG[i][0];
        for (int j = 0; j < 188; j++)
        {
            if ((*map) > threshold)
                (*my_map) = 1;
            else (*my_map) = 0;
            map++;
            my_map++;
        }
    }
}

////////////////////////////////////////////
//���ܣ�������峵ͷ
//���룺
//�����
//��ע��Ҫ�����Լ���ͷ�Ĵ�С�����޸�
///////////////////////////////////////////
void head_clear(void)
{
    uint8_t* my_map;
    for (int i = 85; i >= 80; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 53; j <= 129; j++)
        {
            *(my_map+j) = white;
        }
    }
    for (int i = 80; i >= 70; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 70; j <= 120; j++)
        {
            *(my_map + j) = white;
        }
    }
}

////////////////////////////////////////////
//���ܣ����Ҹ��ڵ�
//���룺�ڵ���
//�������������
//��ע����·��ѹ��
///////////////////////////////////////////
int find_f(int node)
{
    if (f[node] == node)return node;//�ҵ���������ȣ�return
    f[node] = find_f(f[node]);//����Ѱ���Լ��ĸ��ڵ�
    return f[node];
}

////////////////////////////////////////////
//���ܣ���ȡ������ ����ȫ�������ӱ��
//���룺IMG[120][188]
//�����white_range[120]
//��ע��ָ������
///////////////////////////////////////////
void search_white_range()
{
    uint8_t i, j;
    int istart = NEAR_LINE;//������ʼ��
    int iend = FAR_LINE;//������ֹ��
    int tnum = 0;//��ǰ�а�����
    all_connect_num = 0;//������ų�ʼ��
    uint8_t* map = NULL;
    for (i = istart; i >= iend; i--)
    {
        map = &IMG[i][LEFT_SIDE];//ָ�����߼ӿ�����ٶ�
        tnum = 0;
        for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
        {
            if ((*map))//��������߽�
            {
                tnum++;
                if (tnum >= white_num_MAX)break;
                range* now_white = &white_range[i].area[tnum];
                now_white->left = j;//��¼��thum����߽��ֵj

                //��ʼ���һ��һ�����ص�����������ұ߽�
                map++;
                j++;

                while ((*map) && j <= RIGHT_SIDE)
                {
                    map++;
                    j++;
                }
                now_white->right = j - 1;//��¼�ұ߽��ֵj
                now_white->connect_num = ++all_connect_num;//��������һ��������������
            }
        }
        white_range[i].num = tnum;
    }
}

////////////////////////////////////////////
//���ܣ�Ѱ�Ұ�������ͨ�ԣ���ȫ����ͨ�����ӵĽڵ���ˢ����������ȵĽڵ���
//���룺
//�����
//��ע��
///////////////////////////////////////////
void find_all_connect()
{
    //f�����ʼ��
    for (int i = 1; i <= all_connect_num; i++)
        f[i] = i;

    //uΪup dΪdown ��Ϊ��ǰ������������е��������к���������
    //u_num�������а�����
    //u_left�������е�ǰ������߽�
    //u_right�������е�ǰ�����ұ߽�
    //i_u����ǰ�������������ǵ�ǰ���У������У������еĵ�i_u��
    int u_num, i_u, u_left, u_right;
    int d_num, i_d, d_left, d_right;
    all_range* u_white = NULL;
    all_range* d_white = NULL;
    for (int i = NEAR_LINE; i > FAR_LINE; i--)//��Ϊÿ����ÿ���бȽ� ����ѭ����FAR_LINE+1
    {
        u_num = white_range[i - 1].num;
        d_num = white_range[i].num;
        u_white = &white_range[i - 1];
        d_white = &white_range[i];
        i_u = 1; i_d = 1;

        //ѭ������ǰ�л������а��������ľ�Ϊֹ
        while (i_u <= u_num && i_d <= d_num)
        {
            //�����ȱ��棬�����������д�������ҷ���Ч�ʵ�
            u_left = u_white->area[i_u].left;
            u_right = u_white->area[i_u].right;
            d_left = d_white->area[i_d].left;
            d_right = d_white->area[i_d].right;

            if (u_left <= d_right && u_right >= d_left)//�������������ͨ
                f[find_f(u_white->area[i_u].connect_num)] = find_f(d_white->area[i_d].connect_num);//���ڵ�������

            //��ǰ�㷨��������һ�����֪��Ϊɶ������
            if (d_right > u_right)i_u++;
            if (d_right < u_right)i_d++;
            if (d_right == u_right) { i_u++; i_d++; }//û����
        }
    }
}

////////////////////////////////////////////
//���ܣ�Ѱ������
//���룺
//�����
//��ע��
///////////////////////////////////////////
void find_road()
{
    uint8_t istart = NEAR_LINE;
    uint8_t iend = FAR_LINE;
    top_road = NEAR_LINE;//������ߴ������������ȳ�ʼ����Ϊ��ʹ�
    int road_f = -1;//����������ͨ�򸸽ڵ��ţ��ȳ�ʼ��Ϊ-1�����ж��Ƿ��ҵ�����
    int while_range_num = 0, roud_while_range_num = 0;
    all_range* twhite_range = NULL;
    road* tmy_road = NULL;
    //Ѱ������������ͨ��
    // Ѱ�������ĵİ�����
    for (int i = 1; i <= white_range[istart].num; i++)
        if (white_range[istart].area[i].left <= CAMERA_W / 2
            && white_range[istart].area[i].right >= CAMERA_W / 2 && (white_range[istart].area[i].right - white_range[istart].area[i].left) >= 90)
            road_f = find_f(white_range[istart].area[i].connect_num);

    if (road_f == -1)//������û���м䣬��113��ѡһ�������Ϊ���������
    {
        int widthmax = 0, jselect = 1;
        for (int i = 1; i <= white_range[istart].num; i++)
            if (white_range[istart].area[i].right - white_range[istart].area[i].left > widthmax)
            {
                widthmax = white_range[istart].area[i].right - white_range[istart].area[i].left;
                jselect = i;
            }
        road_f = find_f(white_range[istart].area[jselect].connect_num);
    }

    //���������Ѿ��õ�������������ͨ�򸸽ڵ��ţ������������и��ڵ�����road_f�����а������ӽ��������������
    for (int i = istart; i >= iend; i--)
    {
        //�������棬����֮��д�������ҵ�Ч
        twhite_range = &white_range[i];
        tmy_road = &my_road[i];
        while_range_num = twhite_range->num;
        tmy_road->white_num = 0;
        roud_while_range_num = 0;
        for (int j = 1; j <= while_range_num; j++)
        {
            if (find_f(twhite_range->area[j].connect_num) == road_f)
            {
                top_road = i;
                tmy_road->white_num++; roud_while_range_num++;
                tmy_road->connected[roud_while_range_num].left = twhite_range->area[j].left;
                tmy_road->connected[roud_while_range_num].right = twhite_range->area[j].right;
                tmy_road->connected[roud_while_range_num].width = twhite_range->area[j].right - twhite_range->area[j].left;

            }
        }
    }
}

////////////////////////////////////////////
//���ܣ�����������һ�а����ӱ��
//���룺i_start��ʼ��  j_start�������
//������������
//��ע����Ϊ��һ���뱾�������ص����ֶԶ�İ���Ϊѡ������
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
    uint8_t j_return;
    uint8_t j;
    uint8_t width_max = 0;
    uint8_t width_new = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    uint8_t dright, dleft, uright, uleft;
    j_return = MISS;//���û�ҵ������255
    if (j_start > my_road[i_start].white_num)
        return MISS;
    //ѡһ���ص�����
    for (j = 1; j <= my_road[i_start - 1].white_num; j++)
    {
        dleft = my_road[i_start].connected[j_start].left;
        dright = my_road[i_start].connected[j_start].right;
        uleft = my_road[i_start - 1].connected[j].left;
        uright = my_road[i_start - 1].connected[j].right;
        if (//����
            dleft < uright
            &&
            dright > uleft
            )
        {
            //�����ص���С
            if (dleft < uleft) left = uleft;
            else left = dleft;

            if (dright > uright) right = uright;
            else right = dright;

            width_new = right - left + 1;

            if (width_new > width_max)
            {
                width_max = width_new;
                j_return = j;
            }
        }

    }
    return j_return;
}

////////////////////////////////////////////
//���ܣ�ͨ�þ���˫��
//���룺
//�����
//��ע��
///////////////////////////////////////////
void ordinary_two_line(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t j_continue[CAMERA_H];//��һ����ͨ·��
    uint8_t i_start;
    uint8_t i_end;
    uint8_t j_start = MISS;
    int width_max;

    //Ѱ����ʼ�����İ�����
    i_start = NEAR_LINE;
    i_end = FAR_LINE;
    width_max = 0;
    for (j = 1; j <= my_road[i_start].white_num; j++)
    {
        if (my_road[i_start].connected[j].width > width_max)
        {
            width_max = my_road[i_start].connected[j].width;
            j_start = j;
        }
    }
    j_continue[i_start] = j_start;

    //��¼����������
    for (i = i_start; i > i_end; i--)
    {
        //���������Ŵ��ڸ��а����������������Ӵ�֮��MISS
        if (j_continue[i] > my_road[i].white_num)
        {
            j_continue[i - 1] = MISS;
        }
        else
        {
            j_continue[i - 1] = find_continue(i, j_continue[i]);
        }

    }

    //ȫ����ʼ��ΪMISS
    my_memset(left_line, MISS, CAMERA_H);
    my_memset(right_line, MISS, CAMERA_H);


    for (i = i_start; i > i_end; i--)
    {
        if (j_continue[i] <= my_road[i].white_num)
        {
            left_line[i] = my_road[i].connected[j_continue[i]].left;
            right_line[i] = my_road[i].connected[j_continue[i]].right;
            IMG[i][left_line[i]] = blue;
            IMG[i][right_line[i]] = green;
        }
        else
        {
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
}

////////////////////////////////////////////
//���ܣ������ʼ��
//���룺uint8_t* ptr �����׵�ַ, uint8_t num��ʼ����ֵ, uint8_t size�����С
//�����
//��ע����Ϊk66������Ϊmemset��������ȫ�������޷�ʹ�ã������Ҫ�Լ�дһ��my_memset
///////////////////////////////////////////
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size)
{
    uint8_t* p = ptr;
    uint8_t my_num = num;
    uint8_t Size = size;
    for (int i = 0; i < Size; i++, p++)
    {
        *p = my_num;
    }
}
////////////////////////////////////////////
//���ܣ����ߺϳ�
//���룺���ұ߽�
//���������
//��ע��
///////////////////////////////////////////
void get_mid_line(void)
{
    my_memset(mid_line, MISS, CAMERA_H);
    for(int i = NEAR_LINE;i >= FAR_LINE;i--)
        if (left_line[i] != MISS)
        {
            mid_line[i] = (left_line[i] + right_line[i]) / 2;
        }
        else
        {
            mid_line[i] = MISS;
        }

}
////////////////////////////////////////////
//���ܣ�ͼ����������
//���룺
//�����
//��ע��
///////////////////////////////////////////
void image_main()
{
    THRE();

    search_white_range();
    find_all_connect();
    find_road();
    /*���˴�Ϊֹ�������Ѿ��õ������������Ľṹ������my_road[CAMERA_H]*/
    ordinary_two_line();
    get_mid_line();

    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
        if (mid_line[i] != MISS)
            IMG[i][mid_line[i]] = red;

    test_varible[13] = mid_line[50];

}


