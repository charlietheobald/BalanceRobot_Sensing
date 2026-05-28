import processing.serial.*; 
import java.awt.event.KeyEvent; 
import java.io.IOException;

Serial myPort; 
String angle="";
String distance="";
String data="";
String noObject;
float pixsDistance;
int iAngle, iDistance;
int index1=0;
int index2=0;
PFont orcFont;

int maxDistance = 100;
int minDistance = 10;

// --- NEW BUFFER FOR PERSISTENT PLOTTING ---
PGraphics radarCanvas; 

void setup() {
  size(1000, 650); 
  smooth();
  
  // Initialize the off-screen canvas to hold the persistent red grid area and targets
  radarCanvas = createGraphics(width, height);
  radarCanvas.beginDraw();
  radarCanvas.smooth();
  radarCanvas.background(0, 0); // Transparent start
  // Initialize the canvas with the base red protractor shape
  radarCanvas.pushMatrix();
  radarCanvas.translate(width/2, height - height*0.074);
  float maxRadarRadius = width - width*0.0625;
  radarCanvas.fill(150, 5, 5);
  radarCanvas.noStroke();
  radarCanvas.arc(0, 0, maxRadarRadius, maxRadarRadius, PI, TWO_PI);
  radarCanvas.popMatrix();
  radarCanvas.endDraw();

  myPort = new Serial(this, "COM5", 115200); 
  myPort.bufferUntil('.'); 
  orcFont = createFont("Arial", 30);
}

void draw() {
  fill(98,245,31);
  textFont(orcFont);
  
  // Main background black space reset
  noStroke();
  fill(0); 
  rect(0, 0, width, height-height*0.065); 
  
  // 1. Draw the persistent layer (Red background + persistent Green targets)
  image(radarCanvas, 0, 0);
  
  // 2. Overlay the structural grid lines and arcs on top
  drawRadarGrid(); 
  
  // 3. Draw the sweeping line dynamically
  drawLine();
  
  // 4. Update UI text strings
  drawText();
}

void serialEvent (Serial myPort) { 
  data = myPort.readStringUntil('.');
  if (data != null) {
    data = data.substring(0,data.length()-1);
    println(data);
    
    index1 = data.indexOf(","); 
    if (index1 != -1) {
      angle = data.substring(0, index1); 
      distance = data.substring(index1+1, data.length()); 
      
      iAngle = int(angle.trim());
      iDistance = int(distance.trim());
      
      // Update persistent targets directly to the canvas layer whenever data streams in
      drawObjectToCanvas();
    }
  }
}

// Draws the static green grid lines dynamically over the red background every frame
void drawRadarGrid() {
  pushMatrix();
  translate(width/2, height - height*0.074); 
  float maxRadarRadius = width - width*0.0625;
  
  noFill();
  strokeWeight(2);
  stroke(98,245,31); // Grid color
  
  for (int i = 10; i <= maxDistance; i += 10) {
    float arcDiameter = maxRadarRadius * (i / 100.0);
    arc(0, 0, arcDiameter, arcDiameter, PI, TWO_PI);
  }
  
  float lineLength = maxRadarRadius / 2;
  line(-lineLength, 0, lineLength, 0);
  line(0, 0, -lineLength*cos(radians(30)), -lineLength*sin(radians(30)));
  line(0, 0, -lineLength*cos(radians(60)), -lineLength*sin(radians(60)));
  line(0, 0, -lineLength*cos(radians(90)), -lineLength*sin(radians(90)));
  line(0, 0, -lineLength*cos(radians(120)), -lineLength*sin(radians(120)));
  line(0, 0, -lineLength*cos(radians(150)), -lineLength*sin(radians(150)));
  popMatrix();
}

// Draws historical target data directly onto the canvas so they remain saved until overwritten
void drawObjectToCanvas() {
  radarCanvas.beginDraw();
  radarCanvas.pushMatrix();
  radarCanvas.translate(width/2, height - height*0.074); 
  
  float maxRadarRadius = width - width*0.0625;
  float maxLimitPixels = maxRadarRadius / 2;
  
  // --- LAYER RECOVERY / CLEAR TRACE LOOP ---
  // Overwrites the exact slice with the clean red background first to wipe out older reads at this angle
  radarCanvas.strokeWeight(12); // Slightly wider than target line to clean edge artifacts
  radarCanvas.stroke(200, 10, 10); // Red background mask color
  radarCanvas.line(0, 0, maxLimitPixels*cos(radians(iAngle)), -maxLimitPixels*sin(radians(iAngle)));
  
  // --- PLOT NEW GREEN TARGET ---
  radarCanvas.strokeWeight(9);
  radarCanvas.stroke(30, 250, 60); // CHANGED TO GREEN LINES ONLY
  
  // assigns the pixel distance to draw the line
  
  // LINE PLOTTING LOGIC:
  // Draw a green line until the distance that is measured by the sensor
  if(iDistance <= maxDistance){
    pixsDistance = iDistance * (maxLimitPixels / (float)maxDistance); 
  }
  else{
    pixsDistance = maxLimitPixels;
  }
  
   // Plot the green target boundary line directly into our canvas matrix memory
   radarCanvas.line(0, 0, pixsDistance*cos(radians(iAngle)), -pixsDistance*sin(radians(iAngle)));
   radarCanvas.popMatrix();
   radarCanvas.endDraw();
}

void drawLine() {
  pushMatrix();
  strokeWeight(9);
  stroke(30,250,60, 150); // Added slight transparency for an authentic radar sweep aesthetic
  translate(width/2, height - height*0.074); 
  
  float maxRadarRadius = width - width*0.0625;
  float lineLength = maxRadarRadius / 2;
  line(0, 0, lineLength*cos(radians(iAngle)), -lineLength*sin(radians(iAngle))); 
  popMatrix();
}

void drawText() { 
  pushMatrix();
  if(iDistance > minDistance) {
    noObject = "Move";
  }
  else {
    noObject = "Stop";
  }
  
  fill(0,0,0);
  noStroke();
  rect(0, height-height*0.0648, width, height);
  fill(98,245,31);
  
  textSize(20);
  float maxRadarRadius = width - width*0.0625;
  float halfRadar = maxRadarRadius / 2;
  
  text("20cm", (width/2) + (halfRadar * 0.2) - 20, height-height*0.0833);
  text("40cm", (width/2) + (halfRadar * 0.4) - 20, height-height*0.0833);
  text("60cm", (width/2) + (halfRadar * 0.6) - 20, height-height*0.0833);
  text("80cm", (width/2) + (halfRadar * 0.8) - 20, height-height*0.0833);
  text("100cm", (width/2) + halfRadar - 25, height-height*0.0833);
  
  textSize(40);
  text("Object: " + noObject, width-width*0.875, height-height*0.0277);
  text("Angle: " + iAngle +" °", width-width*0.48, height-height*0.0277);
  text("Distance: ", width-width*0.26, height-height*0.0277);
  
  if(iDistance < maxDistance) {
    text("        " + iDistance +" cm", width-width*0.225, height-height*0.0277);
  }
  
  textSize(25);
  fill(98,245,60);
  translate((width-width*0.4994)+width/2*cos(radians(30)),(height-height*0.0907)-width/2*sin(radians(30)));
  rotate(-radians(-60));
  text("30°",0,0);
  resetMatrix();
  translate((width-width*0.503)+width/2*cos(radians(60)),(height-height*0.0888)-width/2*sin(radians(60)));
  rotate(-radians(-30));
  text("60°",0,0);
  resetMatrix();
  translate((width-width*0.507)+width/2*cos(radians(90)),(height-height*0.0833)-width/2*sin(radians(90)));
  rotate(radians(0));
  text("90°",0,0);
  resetMatrix();
  translate(width-width*0.513+width/2*cos(radians(120)),(height-height*0.07129)-width/2*sin(radians(120)));
  rotate(radians(-30));
  text("120°",0,0);
  resetMatrix();
  translate((width-width*0.5104)+width/2*cos(radians(150)),(height-height*0.0574)-width/2*sin(radians(150)));
  rotate(radians(-60));
  text("150°",0,0);
  popMatrix(); 
}
