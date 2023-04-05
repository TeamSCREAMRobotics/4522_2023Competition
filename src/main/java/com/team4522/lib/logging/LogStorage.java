package com.team4522.lib.logging;

import java.util.ArrayList;

public class LogStorage implements Loggable{

	private String mHeader;
	private ArrayList<String> mData = new ArrayList<String>();
	private String mName;

	public LogStorage(String name){
		mName = name;
	}

	@Override
	public String headers() {
		return mHeader;
	}

	@Override
	public ArrayList<String> data() {
		return mData;
	}
	
	@Override
	public void clearData(){
		mData.clear();
	}

	@Override
	public String name() {
		return mName;
	}

	public void addData(String newData){
		mData.add(newData);
	}

	public void setHeader(String header){
		mHeader = header;
	}
}
