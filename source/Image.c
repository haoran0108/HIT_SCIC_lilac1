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

uint8_t *fullBuffer = &mt9v034_image[0][0];
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

uint8_t left_down_point;//Ѱ�����½ǵĹյ������
uint8_t left_up_point;//Ѱ�����ϽǵĹյ������
uint8_t right_down_point;//Ѱ�����½ǵĹյ������
uint8_t right_up_point;//Ѱ�����½ǵĹյ������


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
    int count = 0;
    for (int i = 40; i < 50; i++)
    {
        for (int j = 148; j <= 155; j++)
        {
            count = 0;
            for (int y = i - 2; y <= i + 2; y++)
            {
                for (int x = j - 2; x <= j + 2; x++)
                {
                    map = fullBuffer + 188 * y + x - 1;
                    count = count + (*map);
                }
            }
            map = fullBuffer + 188 * i + j - 1;
            (*map) = count / 25;
        }
    }

    map = fullBuffer;
    for (int i = 0; i < 120; i++)
    {
        my_map = &IMG[i][0];
        for (int j = 0; j < 188; j++)
        {
            if ((*map) > presentTHRE.intValue)
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
        for (int j = 37; j <= 150; j++)
        {
            *(my_map + j) = white;
        }
    }
    for (int i = 80; i >= 70; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 37; j <= 150; j++)
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
                now_white->left = j;

                //��ʼ���һ��һ�����ص�����������ұ߽�
                map++;
                j++;

                while ((*map) && j <= RIGHT_SIDE)
                {
                    map++;
                    j++;
                }
                now_white->right = j - 1;
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
            if (d_right == u_right) { i_u++; i_d++; }
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

    uint8_t upmidline[10];
    uint8_t width[10];
    uint8_t midD_value_min = 100;
    uint8_t t = 0;
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
            //
            width[j] = width_new;
            upmidline[j] = (uleft + uright) / 2;
            if (abs(upmidline[j] - 94) < midD_value_min)
            {
                midD_value_min = abs(upmidline[j] - 94);
                t = j;
            }
            //
            if (width_new > width_max)
            {
                width_max = width_new;
                j_return = j;
            }
        }

    }
    if (j_return != t && width[t] >= 15)
    {
        j_return = t;
    }
    return j_return;
    /*uint8_t j_return;
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
    return j_return;*/
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
            //IMG[i][left_line[i]] = gray;
            //IMG[i][right_line[i]] = purple;
        }
        else
        {
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
    find_point_of_inflection(j_continue);
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
    head_clear();
    search_white_range();
    find_all_connect();
    find_road();
    /*���˴�Ϊֹ�������Ѿ��õ������������Ľṹ������my_road[CAMERA_H]*/
    ordinary_two_line();
    get_mid_line();

    stop();
}

////////////////////////////////////////////
//���ܣ��ҵ��յ�
//���룺
//�����
//��ע��
///////////////////////////////////////////
void find_point_of_inflection(uint8_t j_continue[CAMERA_H]){
    int i;
    int j;

    int flag_left = 1;
    int flag_right = 1;
    left_down_point = 100;
    right_down_point = 100;
    left_up_point = 0;
    right_down_point = 0;

    for (i = 68; i >= 20; i--) {
        if((my_road[i - 1].connected[j_continue[i - 1]].left - my_road[i].connected[j_continue[i]].left <= -3)&&
            //(my_road[i + 1].connected[j_continue[i + 1]].left - my_road[i].connected[j_continue[i]].left >= -3) &&
            //(my_road[i + 1].connected[j_continue[i + 1]].left - my_road[i].connected[j_continue[i]].left <= 3) &&
           (my_road[i - 2].connected[j_continue[i - 2]].left - my_road[i].connected[j_continue[i]].left <= -3)
            ){
            left_down_point = i;
            break;
        }
    }
    if (left_down_point == 20) {
        left_down_point = 91;
    }
    for (i = 68; i >= 20; i--) {
        if ((my_road[i - 1].connected[j_continue[i - 1]].right - my_road[i].connected[j_continue[i]].right >= 3) &&
            //(my_road[i + 1].connected[j_continue[i + 1]].right - my_road[i].connected[j_continue[i]].right <= 3)
            //(my_road[i + 1].connected[j_continue[i + 1]].right - my_road[i].connected[j_continue[i]].right >= -3) &&
            (my_road[i - 2].connected[j_continue[i - 2]].right - my_road[i].connected[j_continue[i]].right >= 3)
            ) {
            right_down_point = i;
            break;
        }
    }
    if (right_down_point == 20) {
        right_down_point = 91;
    }

    for (i = 15; i <=65; i++) {
        if ((//my_road[i + 1].connected[j_continue[i + 1]].left - my_road[i].connected[j_continue[i]].left >= 3 ||
            my_road[i + 1].connected[j_continue[i + 1]].left - my_road[i].connected[j_continue[i]].left <= -3)&&
            (//my_road[i + 2].connected[j_continue[i + 2]].left - my_road[i].connected[j_continue[i]].left >= 3 ||
            my_road[i + 2].connected[j_continue[i + 2]].left - my_road[i].connected[j_continue[i]].left <= -3)
            ) {
            left_up_point = i-1;
            break;
        }
    }
    if (left_up_point == 65) {
        left_down_point = -5;
    }

    for (i = 15; i <= 65; i++) {
        if ((my_road[i + 1].connected[j_continue[i + 1]].right - my_road[i].connected[j_continue[i]].right >= 3 /* ||
            my_road[i + 1].connected[j_continue[i + 1]].right - my_road[i].connected[j_continue[i]].right <= -3*/) &&
            (my_road[i + 2].connected[j_continue[i + 2]].right - my_road[i].connected[j_continue[i]].right >= 3 /*||
                my_road[i + 2].connected[j_continue[i + 2]].right - my_road[i].connected[j_continue[i]].right <= -3*/)
            ) {
            right_up_point = i-1;
            break;
        }

    }
    if (left_up_point == 65) {
        left_down_point = -5;
    }

    find_type_road(j_continue);
}

////////////////////////////////////////////
//���ܣ��жϵ�·����
//���룺
//�����
//��ע��
///////////////////////////////////////////
void find_type_road(uint8_t j_continue[CAMERA_H]) {
    uint8_t i_start = NEAR_LINE;
    uint8_t i_end = FAR_LINE;
    uint8_t flag_left = 1;
    uint8_t flag_right = 1;
    int x_range = 6;
    int y_range = 6;
    int i;
    int j;
    int count;
    int count_black_number;


    float c_of_letf = calculate_curvature(left_down_point, 80, (left_down_point + 80) / 2,
        my_road[left_down_point].connected[j_continue[left_down_point]].left,
        my_road[80].connected[j_continue[80]].left,
        my_road[(left_down_point + 80) / 2].connected[j_continue[(left_down_point + 80) / 2]].left);

    float c_of_right = calculate_curvature(right_down_point, 80, (right_down_point + 80) / 2,
        my_road[right_down_point].connected[j_continue[left_down_point]].right,
        my_road[80].connected[j_continue[80]].right,
        my_road[(right_down_point + 80) / 2].connected[j_continue[(right_down_point + 80) / 2]].right);

    float k_of_down_left = calculate_k(left_down_point, 68, my_road[left_down_point].connected[j_continue[left_down_point]].left, my_road[68].connected[j_continue[68]].left);
    float k_of_down_right= calculate_k(right_down_point, 68, my_road[right_down_point].connected[j_continue[right_down_point]].right, my_road[68].connected[j_continue[68]].right);
    float k_of_up_left = calculate_k(left_up_point, 10, my_road[left_up_point].connected[j_continue[left_up_point]].left, my_road[10].connected[j_continue[10]].left);
    float k_of_up_right = calculate_k(right_up_point, 10, my_road[right_up_point].connected[j_continue[right_up_point]].right, my_road[10].connected[j_continue[10]].right);

    /*for (int m = 0; m < 90; m++) {
        IMG[right_up_point][180 - m] = red;
        IMG[(right_down_point )][180 - m] = green;

        IMG[left_up_point][20 + m] = red;
        IMG[(left_down_point)][20 + m] = green;
    }
    for (int m = 0; m < 80; m++) {
        IMG[m][my_road[left_down_point].connected[j_continue[left_down_point]].left] = blue;
        IMG[m][my_road[left_up_point].connected[j_continue[left_up_point]].left] = purple;
        IMG[m][my_road[right_down_point].connected[j_continue[right_down_point]].right] = blue;
        IMG[m][my_road[right_up_point].connected[j_continue[right_up_point]].right] = purple;
    }*/
    if(!GPIO_Read(P13, 2))
    {
        zebraCount = 0;

    }
    else
    {
         zebraCount++;
         if (zebraCount > 250) zebraCount = 250;
         if (zebraCount % 250 == 0)//5s
         {

             zFlag = 1;
         }

         if (zebraPanduan() == 1 && zFlag == 1)
         {
             zebraCircle++;
             if (zebraCircle >= 2) {
                 zebraCircle = 1;
             }
             zebraCount = 0;
             zFlag = 0;
         }
    }


/*
    //        if(zFlag == 1&&zebraPanduan())
    //        {
    //            zebraCircle++;
    //            zFlag = 0;
    //            zebraCount = 0;
    //        }


    //    count = 0;
    //    int black_flag = 0;
    //    for (i = 70; i >= 10; i--) {
    //        count_black_number = -1;
    //        for (j = 30; j <= 140; j++) {
    //            if (IMG[i][j] == black && IMG[i][j - 1] == black && IMG[i][j + 1] == white && IMG[i][j-2] == black
    //                && IMG[i][j+2] == white) {
    //                count_black_number++;
    //                if (count_black_number >= 5 && count >= 3) {
    //                    black_flag = 1;
    //                    break;
    //                }
    //            }
    //        }
    //        count++;*/
            if (zebraPanduan()==1) {
                for (i = 0; i < 200; i++) {
                    IMG[50][i] = purple;
                }
                left_down_point = -1;
                right_down_point = -1;
                left_up_point = -5;
                right_up_point = -5;

            }



    if (((my_road[left_down_point - 3].connected[j_continue[left_down_point - 3]].width > 95 && my_road[right_down_point - 3].connected[j_continue[right_down_point - 3]].width > 95) ||
        (my_road[left_down_point - 4].connected[j_continue[left_down_point - 4]].width > 95 && my_road[right_down_point - 4].connected[j_continue[right_down_point - 4]].width > 95))&&
        (left_down_point <= 65 && right_down_point <= 65)
        //k_of_down_left * k_of_down_right <= 0 && left_down_point <= 70 && right_down_point <= 70 &&
        //c_of_letf <= 0.4 && c_of_letf >= -0.4 && c_of_right <= 0.4 && c_of_right >= -0.4 &&
        /*(my_road[left_down_point - 5].connected[j_continue[left_down_point - 5]].width > 100 || my_road[right_down_point - 5].connected[j_continue[right_down_point - 5]].width > 100) */ ) {
        //make_unmain_black(j_continue);
        stra_cross_road(j_continue);
    }
    else if (((my_road[left_up_point + 3].connected[j_continue[left_up_point + 3]].width > 95 && my_road[right_up_point + 3].connected[j_continue[right_up_point + 3]].width > 95)||
        (my_road[left_up_point + 4].connected[j_continue[left_up_point + 4]].width > 95 && my_road[right_up_point + 4].connected[j_continue[right_up_point + 4]].width > 95))&&
        (left_up_point >= 10 && right_up_point >= 10)
        /*k_of_up_left * k_of_up_right <= 0 && left_up_point >= 10 && right_up_point >= 10 &&
        //c_of_letf<=0.4 &&c_of_letf>=-0.4 && c_of_right<=0.4 && c_of_right>=-0.4&&
        (my_road[left_up_point + 5].connected[j_continue[left_up_point + 5]].width > 100 || my_road[right_up_point + 5].connected[j_continue[right_up_point + 5]].width > 100)*/) {
        //make_unmain_black(j_continue);
        stra_cross_road2(j_continue);
    }
    else if ((my_road[left_down_point - 3].connected[j_continue[left_down_point - 3]].width > 75 && my_road[left_down_point - 4].connected[j_continue[left_down_point - 4]].width > 75) &&
        ((left_down_point <= 75 && right_down_point > 75 && right_down_point < 80) || (left_down_point - right_down_point <= -5 ))) {
        stra_cross_left_down(j_continue);
    }
    else if ((my_road[right_down_point - 3].connected[j_continue[right_down_point - 3]].width > 75 && my_road[right_down_point - 4].connected[j_continue[right_down_point - 4]].width > 75) &&
        ((right_down_point <= 75 && left_down_point > 75 ) || (left_down_point - right_down_point > 5))) {
        stra_cross_right_down(j_continue);
    }
    /*else if (my_road[79].connected[j_continue[79]].width>180 && my_road[78].connected[j_continue[78]].width > 180 &&
        my_road[30].connected[j_continue[30]].width < 80) {
        stra_cross_left_up(j_continue);
    }
    else if ((my_road[right_up_point + 3].connected[j_continue[right_up_point + 3]].width > 100 && my_road[right_up_point + 4].connected[j_continue[right_up_point + 4]].width > 100) &&
        (right_up_point >= 20  )) {
        stra_cross_right_up(j_continue);
    }*/
    else if (zebraPanduan()) {
        zebra_cross(j_continue);
    }
    else {
        //make_unmain_black(j_continue);
        for (i = i_start; i > i_end; i--)
        {
            if (j_continue[i] <= my_road[i].white_num)
            {
                left_line[i] = my_road[i].connected[j_continue[i]].left;
                right_line[i] = my_road[i].connected[j_continue[i]].right;
                IMG[i][left_line[i]] = gray;
                IMG[i][right_line[i]] = purple;
            }
            else
            {
                left_line[i] = MISS;
                right_line[i] = MISS;
            }
        }
    }

}

////////////////////////////////////////////
//���ܣ����ʼ���
//���룺
//�����
//��ע��
///////////////////////////////////////////
float calculate_curvature(int xa, int xb, int xc, int ya, int yb, int yc) {
    float k;
    int s_of_abc;
    int mid;

    s_of_abc = ((xb - xa) * (yc - ya) - (xc - xa) * (yb - ya)) / 2;

    mid = (xb - xa) * (xb - xa) + (yb - ya)*(yb-ya);
    float l_ab = sqrt(mid);
    mid = (xc - xa) * (xc - xa) + (yc - ya) * (yc - ya);
    float l_ac = sqrt(mid);
    mid = (xb - xc) * (xb - xc) + (yb - yc) * (yb - yc);
    float l_bc = sqrt(mid);

    if (l_ab * l_ac * l_bc == 0) {
        k = 0;
    }
    else {
        k = (float)4 * s_of_abc / (l_ab * l_ac * l_bc);
    }

    return k;
}

////////////////////////////////////////////
//���ܣ�б�ʼ���
//���룺
//�����
//��ע��
///////////////////////////////////////////
float calculate_k(int x1, int x2, int y1, int y2) {

    float k = (float)(x1 - x2) / (y1 - y2);
    return k;
}

////////////////////////////////////////////
//���ܣ�ֱʮ��·�ڲ�ȫ·��
//���룺
//�����
//��ע��������������
///////////////////////////////////////////
void stra_cross_road(uint8_t j_continue[CAMERA_H]) {
    uint8_t i_start = NEAR_LINE;
    uint8_t i_end = FAR_LINE;
    if (right_down_point - left_down_point >= -30 && right_down_point - left_down_point <= 30
        &&
        right_down_point <= 65
        &&
        left_down_point <= 65
        )
    {
        int t1 = my_road[left_down_point].connected[j_continue[left_down_point]].left - my_road[68].connected[j_continue[68]].left;
        int t2 = (-1) * my_road[right_down_point].connected[j_continue[right_down_point]].right + my_road[68].connected[j_continue[68]].right;
        for (int j = left_down_point - 1; j > i_end; j--)
        {
            left_line[j] = my_road[left_down_point].connected[j_continue[left_down_point]].left + (left_down_point - j) * t1 / (68 - left_down_point);
        }
        for (int j = right_down_point - 1; j > i_end; j--)
        {
            right_line[j] = my_road[right_down_point].connected[j_continue[right_down_point]].right - (right_down_point - j) * t2 / (68 - right_down_point);
        }
    }
    for (int i = i_start; i > i_end; i--)
    {
        IMG[i][left_line[i]] = purple;
        IMG[i][right_line[i]] = red;
    }
}
////////////////////////////////////////////
//���ܣ�ֱʮ��·�ڲ�ȫ·��
//���룺
//�����
//��ע��������������
///////////////////////////////////////////
void stra_cross_road2(uint8_t j_continue[CAMERA_H]) {
    uint8_t i_start = NEAR_LINE;
    uint8_t i_end = FAR_LINE;
    if (right_up_point - left_up_point >= -30 && right_up_point - left_up_point <= 30
        &&
        right_up_point >= 10
        &&
        left_up_point >= 10
        )
    {
        int t3 = (-1) * my_road[left_up_point].connected[j_continue[left_up_point]].left + my_road[10].connected[j_continue[10]].left;
        int t4 = my_road[right_up_point].connected[j_continue[right_up_point]].right - my_road[10].connected[j_continue[10]].right;
        for (int j = left_up_point + 1; j < i_start; j++)
        {
            left_line[j] = my_road[left_up_point].connected[j_continue[left_up_point]].left - (j - left_up_point) * t3 / (left_up_point - 10);
        }
        for (int j = right_up_point + 1; j < i_start; j++)
        {
            right_line[j] = my_road[right_up_point].connected[j_continue[right_up_point]].right + (j - right_up_point) * t4 / (right_up_point - 10);
        }
    }
    for (int i = i_start; i > i_end; i--)
    {
        IMG[i][left_line[i]] = blue;
        IMG[i][right_line[i]] = purple;
    }
}

////////////////////////////////////////////
//���ܣ�ֱʮ��·�ڲ�ȫ·��
//���룺
//�����
//��ע�����´�����������
///////////////////////////////////////////
void stra_cross_left_down(uint8_t j_continue[CAMERA_H]) {
    uint8_t i_start = 65;
    uint8_t i_end = 10;

    int t1 = my_road[left_down_point].connected[j_continue[left_down_point]].left - my_road[85].connected[j_continue[85]].left;
    for (int j = left_down_point - 1; j > i_end; j--){
        left_line[j] = my_road[left_down_point].connected[j_continue[left_down_point]].left + (left_down_point - j) * t1 / (85 - left_down_point);
    }
    for (int i = i_start; i > i_end; i--)
    {
        IMG[i][left_line[i]] = blue;
        IMG[i][right_line[i]] = gray;
    }
}

////////////////////////////////////////////
//���ܣ�ֱʮ��·�ڲ�ȫ·��
//���룺
//�����
//��ע�����´�����������
///////////////////////////////////////////
void stra_cross_right_down(uint8_t j_continue[CAMERA_H]) {
    uint8_t i_start = 65;
    uint8_t i_end = 10;
    int t2 = (-1) * my_road[right_down_point].connected[j_continue[right_down_point]].right + my_road[85].connected[j_continue[85]].right;
    for (int j = right_down_point - 1; j > i_end; j--)
    {
        right_line[j] = my_road[right_down_point].connected[j_continue[right_down_point]].right - (right_down_point - j) * t2 / (85 - right_down_point);
    }
    for (int i = i_start; i > i_end; i--)
    {
        IMG[i][left_line[i]] = gray;
        IMG[i][right_line[i]] = red;
    }
}

////////////////////////////////////////////
//���ܣ�ֱʮ��·�ڲ�ȫ·��
//���룺
//�����
//��ע�����´�����������
///////////////////////////////////////////
void stra_cross_left_up(uint8_t j_continue[CAMERA_H]) {
    uint8_t i_start = NEAR_LINE;
    uint8_t i_end = FAR_LINE;
        int t3 = (-1) * my_road[left_up_point].connected[j_continue[left_up_point]].left + my_road[10].connected[j_continue[10]].left;
        for (int j = left_up_point + 1; j < i_start; j++)
        {
            left_line[j] = my_road[left_up_point].connected[j_continue[left_up_point]].left - (j - left_up_point) * t3 / (left_up_point - 10);
        }
    for (int i = i_start; i > i_end; i--)
    {
        IMG[i][left_line[i]] = blue;
        IMG[i][right_line[i]] = gray;
    }
}

////////////////////////////////////////////
//���ܣ�ֱʮ��·�ڲ�ȫ·��
//���룺
//�����
//��ע�����´�����������
///////////////////////////////////////////
void stra_cross_right_up(uint8_t j_continue[CAMERA_H]) {
    uint8_t i_start = NEAR_LINE;
    uint8_t i_end = FAR_LINE;
        int t3 = (-1) * my_road[left_up_point].connected[j_continue[left_up_point]].left + my_road[10].connected[j_continue[10]].left;

        for (int j = left_up_point + 1; j < i_start; j++)
        {
            left_line[j] = my_road[left_up_point].connected[j_continue[left_up_point]].left - (j - left_up_point) * t3 / (left_up_point - 10);
        }
    for (int i = i_start; i > i_end; i--)
    {
        IMG[i][left_line[i]] = red;
        IMG[i][right_line[i]] = gray;
    }
}

////////////////////////////////////////////
//���ܣ�������
//���룺
//�����
//��ע��
///////////////////////////////////////////
void zebra_cross(uint8_t j_continue[CAMERA_H]) {
    uint8_t i_start = NEAR_LINE;
    uint8_t i_end = FAR_LINE;
    int leftside=0;
    int rightside=0;
    int leftflag = 1;
    int rightflag = 1;
    for (int i = 65; i >= 10; i--) {
        for (int j = 22; j <= 90; j++) {
            if (IMG[i][j + 1] == white && IMG[i][j] == black && IMG[i][j - 1] == black) {
                left_line[i] = j;
                break;
            }
        }
    }
    for (int i = 65; i >= 10; i--) {
        for (int j = 170; j >= 90; j--) {
            if (IMG[i][j - 1] == white && IMG[i][j] == black && IMG[i][j + 1] == black) {
                right_line[i] = j;
                break;
            }
        }
    }
    for (int i = i_start; i > i_end; i--){
        IMG[i][left_line[i]] = red;
        IMG[i][right_line[i]] = blue;
    }
}

////////////////////////////////////////////
//���ܣ������ߴ���
//���룺
//�����
//��ע��
///////////////////////////////////////////
void zebra_count() {
    int black_number_flag;
    int count_black_number = 0;
    black_number_flag = 0;
    int black_flag = 0;
    int i;
    int colorFlag = 0;
    for (i = 65; i <= 65; i++) {
        for (int j = 30; j <= 140; j++) {
            if (IMG[i][j] == black && IMG[i][j - 1] == black && IMG[i][j + 1] == white && IMG[i][j - 2] == black
                && IMG[i][j + 2] == white) {
                count_black_number++;
            }
            if (count_black_number >= 5) {
                black_number_flag = 1;
                zebraFlag = 1;
                colorFlag = 1;
                break;
            }
        }
        if (black_number_flag == 1) {
            black_number_flag = 0;
            break;
        }
    }
    int white_flag = 0;
    for (i = 65; i <= 65; i++) {
        for (int j = 30; j <= 140; j++) {
            if (white_range[i].num <= 2 && zebraFlag == 1) {
                white_flag = 1;
                zebraFlag = 0;
                zebraCircle++;
                colorFlag = 2;
                break;
            }
        }
        if (white_flag == 1) {
            white_flag = 0;
            break;
        }
    }

    for (i = 10; i < 150; i++) {
        if (colorFlag == 1) {
            IMG[65][i] = blue;
        }
        else if (colorFlag == 2) {
            IMG[65][i] = red;
        }
    }
}

////////////////////////////////////////////
//���ܣ������������
//���룺
//�����
//��ע��
///////////////////////////////////////////
void make_unmain_black(uint8_t j_continue[CAMERA_H]) {
    int i;
    int j;
    for (i = 0; i <= 100; i++) {
        for (j = 0; j < 180; j++) {
            if (j< my_road[i].connected[j_continue[i]].left || j>my_road[i].connected[j_continue[i]].right) {
                IMG[i][j] = black;
            }
        }
    }
}

////////////////////////////////////////////
//���ܣ��������ж�
//���룺
//�����
//��ע��
///////////////////////////////////////////
int zebraPanduan() {
    int count = 0;
    int count_black_number;
    int i, j;
    int black_flag = 0;
    for (i = 70; i >= 10; i--) {
        count_black_number = -1;
        for (j = 30; j <= 140; j++) {
            if (IMG[i][j] == black && IMG[i][j - 1] == black && IMG[i][j + 1] == white && IMG[i][j - 2] == black
                && IMG[i][j + 2] == white) {
                count_black_number++;
                if (count_black_number >= 5 && count >= 3) {
                    black_flag = 1;
                    break;
                }
            }
        }
        count++;
        if (black_flag == 1) {
            left_down_point = 100;
            right_down_point = 100;
            left_up_point = 0;
            right_down_point = 0;
            break;
        }
    }
    if (count_black_number >= 3 && GPIO_Read(P13, 2)) {
        return 1;
    }
    else {
        return 0;
    }
}
//111
int straightSpeedUp()
{

    int count1 = 0;
    int count2 = 0;
    for (int i = 0; i <= 4; i++)
    {
        for (int j = 88; j <= 100; j++)
        {
            if (IMG[i][j] == white) count1++;
        }
    }
    for (int i = 15; i <= 30; i++)
    {
        if (abs(mid_line[i] - 94) < MIDLINE_DELTA) count2++;
    }
    if (calculate_k(30, 60, left_line[30], left_line[60]) * calculate_k(30, 60, right_line[30], right_line[60]) < 0 &&
        count1 > SPEEDUP_COUNT1 && count2 > SPEEDUP_COUNT2)
    {
        return 1;
    }
    else return 0;
}

void stop()
{
    int count = 0;
    for (int i = 68; i > 40; i--)
    {
        for (int j = 40; j <= 140; j++)
        {
            if (IMG[i][j] == black) count++;
        }
    }
    if (count >= 27 * 101) flagStop = 1;

}


