package com.example.lenovo.azimuthtest.MainClass;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Environment;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.example.lenovo.azimuthtest.CollectTimeSet.CollectTime;
import com.example.lenovo.azimuthtest.CollectTimeSet.FileName;
import com.example.lenovo.azimuthtest.QuaternionClass.FusingClass;
import com.example.lenovo.azimuthtest.R;
import com.example.lenovo.azimuthtest.StepDectClass.StepDectFsm;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

public class MainActivity extends AppCompatActivity implements SensorEventListener{
    private static String TAG="AzimuthTest";
    private SensorManager sensorManager;
    private Sensor accsensor,magsensor,gsensor,grsensor,presensor;
    private TextView tv1,tv2,tv3,tv4,tv5,tv6;
    private Button bt1,bt2;
    float[] Rorate=new float[9];
    float[] OriVal=new float[3];
    float[] accVal=new float[3];
    float[] magVal=new float[3];
    float[] gVal=new float[3];
    float[] grVal=new float[3];
    float pressure=0f;
    private float Epsilon=0.0009765625f;
    private float Threshold=0.5f-Epsilon;
    FusingClass fusingClass;
    private float[] Quaternion=new float[4];
    private float[] Euler_degrees=new float[3];
    private StepDectFsm stepDectFsm;
    private int stepcount=0;
    private float zerosTimes=0f;
    private float azimuthInDegree = 0f;//方位角
    private float lastAzimuthInDegree = 0f;//上一个方位角角度
    private int azimuthUpdateTimes = 0;
    private int zeroTimes = 0;//在求取均值的时候，为了删除不必要的数据输出
    private float allAzimuthInDegree = 0f;
    private float meanAzimuthInDegree = 0f;
    private String[] needed_permission;
    private boolean doWrite=false;
    String fileName="magneticdata";
    String sdPath;
    private float stepLength=0f;
    float[] Euler=new float[3];
    private int update=0;
    boolean isGRa=false,isGYR=false,isMAg=false;
    private int count=0;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        initView();
        requestApplicationPermission();
        fusingClass=new FusingClass();
        //步态探测
        stepDectFsm=new StepDectFsm();
        bt1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                doWrite=true;
                count=count+1;
                fileName=fileName+"_"+count;
                Log.d(TAG,"Start...");
            }
        });
        bt2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                doWrite=false;
                fileName="Azimuthdata";
            }
        });

    }

    private void initView() {
        tv1=(TextView)findViewById(R.id.text);
        tv2=(TextView)findViewById(R.id.text1);
        tv3=(TextView)findViewById(R.id.text2);
        tv4=(TextView)findViewById(R.id.text3);
        tv5=(TextView)findViewById(R.id.text4);
        tv6=(TextView)findViewById(R.id.text5);
        bt1=(Button)findViewById(R.id.button);
        bt2=(Button)findViewById(R.id.button1);
    }
    private void requestApplicationPermission() {
        needed_permission = new String[]{
                Manifest.permission.CHANGE_NETWORK_STATE,
                Manifest.permission.CHANGE_WIFI_STATE,
                Manifest.permission.ACCESS_NETWORK_STATE,
                Manifest.permission.ACCESS_WIFI_STATE,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.BLUETOOTH,
                Manifest.permission.READ_LOGS,
                Manifest.permission.READ_EXTERNAL_STORAGE,
                Manifest.permission.WRITE_EXTERNAL_STORAGE,
                Manifest.permission.MOUNT_UNMOUNT_FILESYSTEMS,
                Manifest.permission.INTERNET,
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_FINE_LOCATION
        };
        boolean permission_ok = true;
        for (String permission : needed_permission) {
            if (ContextCompat.checkSelfPermission(this,
                    permission) != PackageManager.PERMISSION_GRANTED) {
                permission_ok = false;
//                mTextView.append(String.valueOf(permission_ok)+"\n");
            }
        }
        if (!permission_ok) {
            ActivityCompat.requestPermissions(this, needed_permission, 1);
        }
    }
    @Override
    protected void onDestroy() {
        super.onDestroy();
        sensorManager.unregisterListener(this);
    }
    //传感器注册操作
    @Override

    protected void onResume() {
        super.onResume();
        sensorManager=(SensorManager)getSystemService(Context.SENSOR_SERVICE);
        accsensor=sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magsensor=sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        gsensor=sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        grsensor=sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        presensor=sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
        sensorManager.registerListener(this,accsensor,CollectTime.COLLECT_NORMAL);
        sensorManager.registerListener(this,magsensor,CollectTime.COLLECT_NORMAL);
        sensorManager.registerListener(this,gsensor,CollectTime.COLLECT_NORMAL);
        sensorManager.registerListener(this,grsensor,CollectTime.COLLECT_NORMAL);
        sensorManager.registerListener(this,presensor,CollectTime.COLLECT_NORMAL);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        String string="";
        switch (event.sensor.getType()){
            case Sensor.TYPE_ACCELEROMETER:
                accVal=event.values.clone();
                if(stepDectFsm.StepDect(accVal)){
                    stepLength=stepDectFsm.getStepLength();
                    stepcount++;
                    tv1.setText("Step number is : "+stepcount);
                    tv2.setText("Step length is : "+stepLength);
                }
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                magVal=event.values.clone();
                isMAg=true;
                tv4.setText("磁力x方向："+magVal[0]);
                tv5.setText("磁力y方向："+magVal[1]);
                tv6.setText("磁力z方向："+magVal[2]);
                update++;
                string=string+magVal[0]+" "+magVal[1]+" "+magVal[2]+" ";
                break;
            case Sensor.TYPE_GRAVITY:
                gVal=event.values.clone();
                isGRa=true;
                string=string+gVal[0]+" "+gVal[1]+" "+gVal[2]+" ";
//                tv4.setText("重力加速度x方向："+gVal[0]);
//                tv5.setText("重力加速度y方向："+gVal[1]);
//                tv6.setText("重力加速度z方向："+gVal[2]);sb.append(magVal[0]).append(',').append(magVal[1]).append(',').append(magVal[2]).append(',');
                update++;
                break;
            case Sensor.TYPE_GYROSCOPE:
                grVal=event.values.clone();
                update++;
                isGYR=true;
                string=string+grVal[0]+" "+grVal[1]+" "+grVal[2]+" ";
                break;
//            case Sensor.TYPE_PRESSURE:
//                pressure=event.values[0];
//                Log.i("TAG-pressure: ",""+pressure);
            default:
                break;
        }
//        if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
//            gVal = event.values.clone();
//            isGRa=true;
//        }
//        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
//            grVal = event.values.clone();
//            isGYR=true;
//        }
//        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
//            magVal = event.values.clone();
//            isMAg=true;
//        }
        float Azimuth=0f,Poitch=0f,Roll=0f;
        //调用方法
//        if(isGRa&&isGYR&&isMAg) {
//            Quaternion = fusingClass.AHRSupdate(grVal[0], grVal[1], grVal[2], gVal[0], gVal[1], gVal[2], magVal[0], magVal[1], magVal[2]);
//            QuaternionToEuler(Quaternion);
//            Euler_degrees=Euler.clone();
//            isGRa=false;
//            isGYR=false;
//            isMAg=false;
//            Log.i(TAG,"data"+","+gVal[0]+","+gVal[1]+","+gVal[2]+","+
//                    grVal[0]+","+grVal[1]+","+grVal[2]+","+magVal[0]+","+magVal[1]+","+magVal[2]);
//            Azimuth=(float)Math.toDegrees(Euler_degrees[2])+90.0f;
//            if(Azimuth<0){
//                Azimuth=360+Azimuth;
//                Azimuth=Azimuth-4.5f;//徐州地磁偏角
//                tv3.setText("Azimuth is : "+Azimuth);
//            }else{
//                Azimuth=Azimuth-4.5f;
//                if(Azimuth<0){
//                    Azimuth=0f;
//                }
//                tv3.setText("Azimuth is : "+Azimuth);
//            }
//            //此处的校正是根据实际的调试结果校正的，具体原因需要后续研究....
////            if(Azimuth>=0&&Azimuth<=270){
////                Azimuth=100+Azimuth;
////                tv3.setText("Azimuth is : "+Azimuth);
////            }else {
////                Azimuth=Azimuth-270;
////                if(Azimuth>0&&Azimuth<=180){
////                    Azimuth=Azimuth-10;
////                }
////                tv3.setText("Azimuth is : "+Azimuth);
////            }
//            Log.i(TAG," "+Azimuth);
//            String message=null;
//            //float Azimuth=getAzimuthInDegree(gVal,magVal);
//            message=""+Azimuth+" "+stepLength+"\n";
//            //把方位和步长数据写入
//            if(doWrite){
//                WriteFileSdcard(message);
//            }
//        }

        if(isGRa&&isGYR&&isMAg){
            SensorManager.getRotationMatrix(Rorate,null,gVal,magVal);
            SensorManager.getOrientation(Rorate,OriVal);
            isGRa=false;
            isGYR=false;
            isMAg=false;
            Azimuth= (float) Math.toDegrees(OriVal[0]);
            Poitch=(float)Math.toDegrees(OriVal[1]);
            Roll= (float) Math.toDegrees(OriVal[2]);
            if(Math.toDegrees(OriVal[0])<0){
                Azimuth=360+Azimuth;
                tv3.setText("Azimuth is : "+Azimuth);
            }else{
                tv3.setText("Azimuth is : "+Azimuth);
            }
            Log.i(TAG," "+Roll);
//            if(Math.toDegrees(Euler_degrees[0])<0){
//                Azimuth=360+Azimuth;
//                tv3.setText("Azimuth is : "+Azimuth);
//            }else {
//                tv3.setText("Azimuth is : "+Azimuth);
//            }
//            tv3.setText("Azimuth is : "+Euler_degrees[0]);
            //文件操作
//            String message=null;
//            //float Azimuth=getAzimuthInDegree(gVal,magVal);
//            message=""+Azimuth+" "+stepLength+"\n";
            string=string+Azimuth+" "+Poitch+" "+Roll+" "+stepLength+"\n";
            //把方位和步长数据写入
            if(doWrite){
                WriteFileSdcard(string);
            }
            tv3.setText("Azimuth is : "+Azimuth);
        }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    private float getAzimuthInDegree(float[] accVal,float[] magVal){
        float azimuth=0.0f;
        if (accVal!= null && magVal!= null) {
            azimuthUpdateTimes++; //计算航向角更新的次数
            if (accVal.equals(magVal)) {
                azimuthInDegree = lastAzimuthInDegree;
            } else {
                SensorManager.getRotationMatrix(Rorate, null, accVal,magVal);
                SensorManager.getOrientation(Rorate,OriVal);
                float azimuthInRad = OriVal[0];
                if (azimuthInRad < 0) {
                    azimuthInRad += 2 * Math.PI;
                }
                azimuthInDegree = (float) Math.toDegrees(azimuthInRad) - 5.9f;//这里先不用磁偏纠正,石家庄的磁偏角是西偏5.9
            }
            lastAzimuthInDegree = azimuthInDegree;
            if (Math.abs(azimuthInDegree - 0) < 0.0001) {
                zeroTimes++; //判断为0的情况
            }
            allAzimuthInDegree += azimuthInDegree;
//            mAccValues = null;
//            mMagValues = null;
            if (azimuthUpdateTimes >= 5) {
                if (zeroTimes == 5) {
                    meanAzimuthInDegree = 0f;
                } else {
                    meanAzimuthInDegree = allAzimuthInDegree / (azimuthUpdateTimes - zeroTimes);
                }
                allAzimuthInDegree = 0f;
                azimuthUpdateTimes = 0;
                zeroTimes = 0;
//                double tmp=0.5+0.1*Math.random()-0.15*Math.random();
                azimuth=meanAzimuthInDegree;

            }
        }

        return azimuth;
    }
    //四元数转欧拉角
    public float[] QuaternionToEuler(float[] Q){
        //这里传入的四元数应当是归一化的，不是归一化的要用另外一种代码
        //float[] Euler=new float[3];
        float TEST=Q[0]*Q[1]+Q[2]*Q[3];
        //奇异姿态，俯仰角正负90
        if(TEST>Threshold){
            Euler[2]= (float) (2*Math.atan2(Q[0],Q[3]));
            Euler[1]= (float) (Math.PI/2);
            Euler[0]=0.0f;
            return Euler;
        }
        if(TEST<-Threshold){
            Euler[2]= -(float) (2*Math.atan2(Q[0],Q[3]));
            Euler[1]= -(float) (Math.PI/2);
            Euler[0]=0.0f;
            return Euler;
        }
        float sqx=Q[0]*Q[0];
        float sqy=Q[1]*Q[1];
        float sqz=Q[2]*Q[2];
        Euler[0]= (float) Math.atan2(2*Q[1]*Q[3]-2*Q[0]*Q[2], 1 - 2*sqy - 2*sqz);
        Euler[1]= (float) Math.asin(2*TEST);
        Euler[2]=(float) Math.atan2(2*Q[0]*Q[3]-2*Q[1]*Q[2],1-2*sqx-2*sqz);
        return Euler;
    }

    private void WriteFileSdcard(String message) {
        try{
            //创建文件夹
            sdPath = Environment.getExternalStorageDirectory().getAbsolutePath()+ File.separator;
            File file = new File(sdPath+ FileName.str+File.separator);
            if(!file.exists()){
                file.mkdir();
            }
            //创建文件并写入
            File file1=new File(sdPath+FileName.str+File.separator+fileName+".txt");
            if (!file1.exists()) {
                file1.createNewFile();
            }
            FileOutputStream fos = new FileOutputStream(file1,true);
            fos.write(message.getBytes());
            fos.close();
        }catch (IOException e){
            e.printStackTrace();
        }
    }
}
