package com.team4522.lib.logging;

import java.util.ArrayList;

public interface Loggable {

	String headers();

	ArrayList<String> data();
	
	void clearData();

	String name();

}