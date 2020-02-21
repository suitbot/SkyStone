package org.firstinspires.ftc.suitbots;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

@TeleOp(name = "hello world")
public class HelloWorld extends OpMode {
    PrintWriter pw;

    @Override
    public void init() {
        try {
            pw = new PrintWriter(new File(Environment.getExternalStorageDirectory(), "hello.txt"));
        } catch (IOException ioe) {
            throw new RuntimeException(ioe);
        }
    }

    @Override
    public void start() {
        super.start();
        pw.println("hello world");
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        pw.close();
        super.stop();
    }
}
