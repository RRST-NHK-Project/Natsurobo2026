
    #ifndef CRANEGAME_HPP
    #define CRANEGAME_HPP

        //同じ基板で2つの機構を動かすつもりなので機構ごとに一応ファイルを分けておく。ここでは小ウナギ捕獲機構に関わる機能のみ記述する。
        

            //主にいじることになる定数
            const int servo_init_deg = 0; //サーボ初期角度（仮）
            const int servo_move_deg = 90; //サーボ展開角度（仮）

            const int motor_pow = 70; //当然仮の値

            //このhppで定義する関数


        /*
        マイコンに送信される配列"data_"
        debug: 機能未割り当て, MD: モータードライバー, TR: トランジスタ
        | data[n] | 詳細 | 範囲 |
        | ---- | ---- | ---- |
        | data[0] | debug | 0 or 1 |
        | data[1] | MD1 | -100 ~ 100 |
        | data[2] | MD2 | -100 ~ 100 |
        | data[3] | MD3 | -100 ~ 100 |
        | data[4] | MD4 | -100 ~ 100 |
        | data[5] | MD5 | -100 ~ 100 |
        | data[6] | MD6 | -100 ~ 100 |
        | data[7] | MD7 | -100 ~ 100 |
        | data[8] | MD8 | -100 ~ 100 |
        | data[9] | Servo1 | 0 ~ 270 |
        | data[10] | Servo2 | 0 ~ 270 |
        | data[11] | Servo3 | 0 ~ 270 |
        | data[12] | Servo4 | 0 ~ 270 |
        | data[13] | Servo5 | 0 ~ 270 |
        | data[14] | Servo6 | 0 ~ 270 |
        | data[15] | Servo7 | 0 ~ 270 |
        | data[16] | Servo8 | 0 ~ 270 |
        | data[17] | TR1 | 0 or 1|
        | data[18] | TR2 | 0 or 1|
        | data[19] | TR3 | 0 or 1|
        | data[20] | TR4 | 0 or 1|
        | data[21] | TR5 | 0 or 1|
        | data[22] | TR6 | 0 or 1|
        | data[23] | TR7 | 0 or 1|
        | data[24] | TR8 | 0 or 1|
        */

        int unaginobori2026::cranegame_servo(){

            if(SQUARE_count %2 == 1){
                return servo_init_deg;
            }
            else if(SQUARE_count %2 == 0){
                return servo_move_deg;
            }

        }

        int unaginobori2026::cranegame_motor(){
            
            if(TRIANGLE){
                return motor_pow;
            }
            else if(CROSS){
                return -motor_pow;
            }
            else
            return 0;
        }

        bool unaginobori2026::cranegame_tr(){
            
            if(SQUARE_count %2 == 0){
                return false;
            }
            else if(SQUARE_count %2 == 1){
                return true;
            }

        }



#endif  

