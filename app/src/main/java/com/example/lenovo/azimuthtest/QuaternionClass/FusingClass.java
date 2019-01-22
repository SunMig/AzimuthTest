package com.example.lenovo.azimuthtest.QuaternionClass;

import com.example.lenovo.azimuthtest.CollectTimeSet.CollectTime;

/**
 * Created by Lenovo on 2018/11/5.
 */

public class FusingClass {
    private float Kp=2.0f;
    private float Ki=0.005f;
    private float halfT=0.25f;
    private float q0=1,q1=0,q2=0,q3=0;
    float exInt=0,eyInt=0,ezInt=0;//表示误差的积分项
    public FusingClass(){

    }
    //融合传感器数据计算四元数,MEMS传感器数据
    public float[] AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz){
        //g表示陀螺仪的度数，指飞行器绕自身x,y,z轴正向旋转的角速度
        //a表示加速度计度数，指飞行器三轴的加速度数据
        //m表示磁传感器度数，指飞行器三轴的磁数据
        float norm;
        float hx,hy,hz,bx,bz;
        float vx,vy,vz,wx,wy,wz;
        float ex,ey,ez;

        float q0q0=q0*q0;
        float q0q1=q0*q1;
        float q0q2=q0*q2;
        float q0q3=q0*q3;
        float q1q1=q1*q1;
        float q1q2=q1*q2;
        float q1q3=q1*q3;
        float q2q2=q2*q2;
        float q2q3=q2*q3;
        float q3q3=q3*q3;
        //加速度数据归一化
        norm = (float) Math.sqrt(ax*ax + ay*ay + az*az);
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;
        //磁场数据归一化
        norm = (float) Math.sqrt(mx*mx + my*my + mz*mz);
        mx = mx / norm;
        my = my / norm;
        mz = mz / norm;
        //hx, hy, hz 表示将飞行器参考系上的地磁矢量转换到地理坐标系（参考坐标系）后的矢量
        hx = (float) (2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2));
        hy = (float) (2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1));
        hz = (float) (2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2));
        bx = (float) Math.sqrt((hx*hx) + (hy*hy));//bx越接近hx，则整个估计姿态与电子罗盘测得的姿态越重合。
        bz = hz;

        //vx, vy, vz为将标准单位重力转换到飞行器参考系后各个坐标轴上的分量。（重力加速度）
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        //wx, wy, wz为将 bx与bz又重新转换到飞行器参考系后各个坐标轴上的分量
        wx = (float) (2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2));
        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
        wz = (float) (2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2));

        //ex, ey, ez 为向量a和向量m，与向量v和向量w的外积的在飞行器参考系的向量。该向量描述了a和m与v和w的偏差程度
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

        //对误差进行积分
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;

        //用误差的积分和误差本身与Kp的乘积的和对角速度进行补偿
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;

        //利用龙格-库格法求出四元数的值，此值由补偿后的角速度求出
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
        //对四元数进行归一化
        norm = (float) (Math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3));
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
        float[] Quaternion={q0,q1,q2,q3};
        return  Quaternion;
    }

    //融合传感器数据计算四元数,MEMS传感器数据
    public float[] AHRSupdate(float[] grVal,float[] gVal,float[] mVal){
        //g表示陀螺仪的度数，指飞行器绕自身x,y,z轴正向旋转的角速度
        //a表示加速度计度数，指飞行器三轴的加速度数据
        //m表示磁传感器度数，指飞行器三轴的磁数据
        float norm;
        float hx,hy,hz,bx,bz;
        float vx,vy,vz,wx,wy,wz;
        float ex,ey,ez;
        float gx,gy,gz,ax,ay,az,mx,my,mz;
        //陀螺仪数据
        gx=grVal[0];
        gy=grVal[1];
        gz=grVal[2];
        //重力加速度
        ax=gVal[0];
        ay=gVal[1];
        az=gVal[2];
        //地磁数据
        mx=mVal[0];
        my=mVal[1];
        mz=mVal[2];

        float q0q0=q0*q0;
        float q0q1=q0*q1;
        float q0q2=q0*q2;
        float q0q3=q0*q3;
        float q1q1=q1*q1;
        float q1q2=q1*q2;
        float q1q3=q1*q3;
        float q2q2=q2*q2;
        float q2q3=q2*q3;
        float q3q3=q3*q3;

        //加速度数据归一化
        norm = (float) Math.sqrt(ax*ax + ay*ay + az*az);
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;
        //磁场数据归一化
        norm = (float) Math.sqrt(mx*mx + my*my + mz*mz);
        mx = mx / norm;
        my = my / norm;
        mz = mz / norm;
        //hx, hy, hz 表示将飞行器参考系上的地磁矢量转换到地理坐标系（参考坐标系）后的矢量
        hx = (float) (2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2));
        hy = (float) (2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1));
        hz = (float) (2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2));
        bx = (float) Math.sqrt((hx*hx) + (hy*hy));//bx越接近hx，则整个估计姿态与电子罗盘测得的姿态越重合。
        bz = hz;

        //vx, vy, vz为将标准单位重力转换到飞行器参考系后各个坐标轴上的分量。（重力加速度）
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        //wx, wy, wz为将 bx与bz又重新转换到飞行器参考系后各个坐标轴上的分量
        wx = (float) (2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2));
        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
        wz = (float) (2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2));

        //ex, ey, ez 为向量a和向量m，与向量v和向量w的外积的在飞行器参考系的向量。该向量描述了a和m与v和w的偏差程度
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

        //对误差进行积分
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;

        //用误差的积分和误差本身与Kp的乘积的和对角速度进行补偿
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;

        //利用龙格-库格法求出四元数的值，此值由补偿后的角速度求出
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
        //对四元数进行归一化
        norm = (float) (Math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3));
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
        float[] Quaternion={q0,q1,q2,q3};
        return  Quaternion;
    }
}
