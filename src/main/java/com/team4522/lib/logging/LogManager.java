package com.team4522.lib.logging;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.TimeZone;

public class LogManager {
	private List<Loggable> mLoggables = new ArrayList<Loggable>();
	private List<FileWriter> mWriters = new ArrayList<FileWriter>();
	private String mRootDirectory;
	private String mDirectory;
	private boolean mLog;
	
	private static LogManager mInstance = null;
	public static LogManager getInstance(){
		if(mInstance == null){
			mInstance = new LogManager();
		}
		return mInstance;
	}

	private LogManager(){
		mRootDirectory = "/home/lvuser/logs";
		mLog = false;
	}

	public void enable(){
		mLog = true;
		open();
	}

	public void disable(){
		mLog = false;
		//close();
	}

	public void registerLoggable(Loggable loggable){
		mLoggables.add(loggable);
	}

	public void open(){
		if(!mLog) return;

		File Directory = new File(mRootDirectory);
            if  (! Directory.isDirectory()) {
                Directory.mkdir();
            }

		Date unformattedTime = new Date(System.currentTimeMillis());
		DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
		dateFormat.setTimeZone(TimeZone.getTimeZone("CST"));

		mDirectory = mRootDirectory + "/" + dateFormat.format(unformattedTime);
		File newDirectory = new File(mDirectory);
		System.out.print("initial directory: " + newDirectory.mkdir());

		for(int i = 0; i < mLoggables.size(); i++){
			Loggable logger = mLoggables.get(i);
			try {
				System.out.println(logger.name());
				FileWriter writer = new FileWriter(mDirectory + "/" + logger.name() + ".log");
				mWriters.add(writer);
				writer.write(logger.headers() + '\n');
			} catch (IOException e) {
				e.printStackTrace();
			}	
		}

	}

	public void log(){//TODO bug if addData is called twice in a loop
		if(!mLog) return;
		for(int i = 0; i < mLoggables.size(); i++){

			Loggable logger = mLoggables.get(i);
			FileWriter writer = mWriters.get(i);

			List<String> data = logger.data();
			for(String d : data){
				try {
					writer.write(d + '\n');
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
			logger.clearData();
			
		}
	}
	
	public void close(){
		if(!mLog) return;
		log();
		for(FileWriter writer : mWriters){
			try {
				writer.close();
			} catch (IOException e) {
                e.printStackTrace();
			}
		}
	}
}