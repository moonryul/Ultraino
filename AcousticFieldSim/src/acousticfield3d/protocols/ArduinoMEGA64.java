package acousticfield3d.protocols;

import acousticfield3d.gui.MainForm;
import acousticfield3d.math.M;
import acousticfield3d.simulation.AnimKeyFrame;
import acousticfield3d.simulation.TransState;
import acousticfield3d.simulation.Transducer;
import acousticfield3d.utils.ArrayUtils;
import acousticfield3d.utils.TextFrame;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 *
 * @author am14010
 */
public class ArduinoMEGA64 extends ArduinoNano{
    final static byte COMMAND_COMMIT_DURATIONS = 0x10;
    
    @Override
    public int getDivs() {
        return 10;
    }

    @Override
    public int getSpeed() {
        return 115200;
    }
    
    public int getNPorts() {
        return 10;
    }
    
    public int getSignalsPerBoard(){
        return 64;
    }
    
    
    public byte[] calcDataBytes(final AnimKeyFrame key) {
        //TODO refactor this shameful repetition of code
        final int nTrans = Transducer.getMaxPin(key.getTransAmplitudes().keySet()) + 1;
        final int signalsPerBoard = getSignalsPerBoard();
        final int nBoards = (nTrans - 1) / signalsPerBoard + 1;
        assert (nBoards < 15);

        final int nDivs = getDivs();
        final int nPorts = getNPorts();

        final int bytesPerBoard = nDivs * nPorts; // = 100 = 10 * 10
        byte[] data = new byte[nBoards * bytesPerBoard]; // = 100, with nBoards=1
        
        for ( Transducer t : key.getTransAmplitudes().keySet() ) {
                        
            final int n = t.getDriverPinNumber();
            // n is the position of the "pin" associated with transducer t.
            if (n >= 0) {
                final float amplitude = key.getTransAmplitudes().get(t);
                final float fphase = key.getTransPhases().get(t);
                
                        
                final int board = n / signalsPerBoard;
                final int softwarePin = n % signalsPerBoard; // signalsPerBoard = 64 = 8 * 8

                final int hardwarePin = PORT_MAPPING[softwarePin];
                final int phaseCompensation = PHASE_COMPENSATION[softwarePin];

                final int targetByte = hardwarePin / 8; // the bit for hardwarePin will be in 
                  // "targetByte" position in data byte array.
                final byte value = (byte) ((1 << (hardwarePin % 8)) & 0xFF);
                 // hardwarePin % 8 will refer to the bit position within the target byte.
                 // 1 << (hardwarePin % 8) will produce the bit string in which the position
                 // with 1 will be the bit position of the hardWarePin
                final int phase = Transducer.calcDiscPhase( fphase + t.getPhaseCorrection(), nDivs);

                //TODO the divs to amplitude is not going to be linear but it will do for the moment
                final int ampDivs = M.iclamp(Math.round(amplitude * nDivs / 2), 0, nDivs);
                // is ampDvis the duty cycle of PWM?
                for (int i = 0; i < ampDivs; ++i) {
                    final int d = (i + phase + phaseCompensation) % nDivs;
                    
                    data[ board * bytesPerBoard + targetByte + d * nDivs] |= value;
                }
            }// if (n >= 0) 
        }//   for ( Transducer t : key.getTransAmplitudes().keySet() )
        
        return data;
    }//byte[] calcDataBytes(final AnimKeyFrame key) 
    
    @Override
    public void sendAnim(final List<AnimKeyFrame> keyFrames) {
        if(serial == null){
            return;
        }
        
        //send patterns
        final List<AnimKeyFrame> frames = keyFrames;
        final ArrayList<Integer> durations = new ArrayList<>();


        for (AnimKeyFrame k : frames) { 
            // the number of frames can be larger than 32, which is
            // N_PATTERNS defined in Arduino; This number is chosen because of the memory limit
            // of Arduino.
            
            final float duration = k.getDuration();
            //if (duration > 0.0f){
            durations.add((int) duration); 
                // the maximum size of durations[] = 32 =N_PATTERNS which is defined by ArduinoMega64.ino

            final AnimKeyFrame akf = k;
            
            final int nTrans = Transducer.getMaxPin(k.getTransAmplitudes().keySet()) + 1;
            
            final int signalsPerBoard = getSignalsPerBoard(); // == 64 = 8 * 8
            final int nBoards = (nTrans - 1) / signalsPerBoard + 1;
            assert (nBoards < 15);

            final int nDivs = getDivs();
            final int nPorts = getNPorts();

            final int bytesPerBoard = nDivs * nPorts; // = 100 = 10 * 10
            
            // Note the different sizes of signalsPerBoard and bytesPerBoard

            byte[] data = calcDataBytes(k); 
              // calcuate the amplitudes and phases of the transducers for the current frame

              // One (the current) frame (pattern) has data for all boards
            for (int i = 0; i < nBoards; ++i) {
                for (int j = 0; j < bytesPerBoard; ++j) {
                    int dataIndex = i * bytesPerBoard + j; // dataIndex =0...,99, when i=0
                    serial.writeByte((data[dataIndex] & 0xF0) | (i + 1));
                    serial.writeByte(((data[dataIndex] << 4) & 0xF0) | (i + 1));
                }
            }

        }//for (AnimKeyFrame k : frames) 
        
        
        //send durations
        durations.add(0); // Insert 0 at the end of the duration array. It means the stop frame of
                          // the animation
        sendDurations( ArrayUtils.toArray(durations) );
    }//sendAnim(final List<AnimKeyFrame> keyFrames)
    
    @Override
    //  TransControlPanel.java will execute device.switchBuffers() 
    //   after sendPattern() is executed
    public void sendPattern(final List<Transducer> transducers) {
        
        // Send ONE pattern (period) whose size is N_DVIS * N_PORTS = 10 * 10
        
       if(serial == null){
            return;
        }
      
       // Calculate Data Bytes: the same as calcDateBytes()
       final int nTrans = Transducer.getMaxPin(transducers) + 1;
       final int signalsPerBoard = getSignalsPerBoard(); // 64 = 8 * 8
       final int nBoards = (nTrans-1) / signalsPerBoard + 1;
       if (nBoards >= 15){
           //TODO log error
           return;
       }
       
       final int nDivs = getDivs();
       final int nPorts = getNPorts();
       
       final int bytesPerBoard = nDivs * nPorts;
       byte[] data = new byte[nBoards * bytesPerBoard]; // data = byte[100]= byte[10*10]
       for(Transducer t : transducers){
           final int n = t.getDriverPinNumber();
                     // n="software pin" assigned to transducer t
           if(n >= 0){
               final int board = n / signalsPerBoard; //== 0..63/64 = 0
               final int softwarePin = n % signalsPerBoard; // n%64=0,1,2,..63
               
               final int hardwarePin = PORT_MAPPING[softwarePin];
               final int phaseCompensation = PHASE_COMPENSATION[softwarePin];
               
               final int targetByte = hardwarePin / 8; // = port index (byte index) of hardware pin
               final byte pinIndex = (byte)((1 << (hardwarePin % 8)) & 0xFF); 
                   // = pin (bit) index within the port of hardware pin
               
               final int phase = Transducer.calcDiscPhase(t.getPhase() + t.getPhaseCorrection(), nDivs);
               //TODO the divs to amplitude is not going to be linear but it will do for the moment
               final int ampDivs = M.iclamp( Math.round(t.getpAmplitude() * nDivs / 2), 0, nDivs );
               
               for (int i = 0; i < ampDivs; ++i) {
                   final int d = (i + (phase + phaseCompensation) ) % nDivs;
                   data[ board* bytesPerBoard + targetByte + d * nDivs] |= pinIndex;                   
                
               }
               // data[0+targetByte], data[10+targetByte], data[2*10+targetByte],
               // ,....,data[9*10+targetByte], where targetByte goes from 0 to 9
           }
       }//   for(Transducer t : transducers)
       // end of   Calculate Data Bytes: the same as calcDateBytes()
       
       // Send pattern byte commands for all the patterns of al boards
       for(int i = 0; i < nBoards; ++i){ // nBoards=1 => one pattern on one board
           for(int j = 0; j < bytesPerBoard; ++j){ // j =0..99
               int dataIndex = i*bytesPerBoard + j;
                serial.writeByte( (data[dataIndex] & 0xF0) | (i+1) );
                serial.writeByte( ((data[dataIndex] << 4) & 0xF0) | (i+1)  );
           }
       }
       
       sendDurations( new int[]{1,0} );
       // durations[0]=1; durations[1]=0
       // This makes the first and one pattern (period) repeated  indefinitely
       // because the next pattern (frame) is the stop frame (duration being 0)
       // and the control returns to the first  pattern when we reach the stop frame
       
    }//sendPattern(final List<Transducer> transducers)
    
    
    @Override
    public void sendDurations(final int[] durations){
        final int n = durations.length;
        final int COMMAND = (byte)0x30; //XX110000; add duration command
         
        //(bReceived & 0x11000000) >> (durationsWrittingIndex % 4 * 2)
        
        for(int i = 0; i < n; ++i){
            final int d = durations[i];
            serial.writeByte( COMMAND | ((d << 0) & 0xC0) );
            serial.writeByte( COMMAND | ((d << 2) & 0xC0) );
            serial.writeByte( COMMAND | ((d << 4) & 0xC0) );
            serial.writeByte( COMMAND | ((d << 6) & 0xC0) );
        }
        
        //commit durations
        serial.writeByte( COMMAND_COMMIT_DURATIONS );
        //  COMMAND_COMMITDURATIONS = 0b00010000 = swap Durations 
         // It  makes the rewly received durations to be used by ArduinoMega
        
        serial.flush();
    }//sendDurations(final int[] durations)
    
    //sorry, repeated code everywhere
    public static void exportAnimation(MainForm mf) {
        final List<AnimKeyFrame> frames = mf.animPanel.getCurrentAnimation().getKeyFrames().getElements();

        ArduinoMEGA64 ins = new ArduinoMEGA64();

        final StringBuilder sb = new StringBuilder();
        sb.append("{");

        final int nKeys = frames.size();
        int ik = 0;
        for (AnimKeyFrame k : frames) {
            final byte[] data = ins.calcDataBytes( k );
            final int l = data.length;
            for (int in = 0; in < l; ++in) {
                sb.append("0x" + Integer.toHexString(data[in] & 0xFF));
                if (in != l - 1) {
                    sb.append(",");
                }

            }
            if (ik != nKeys - 1) {
                sb.append("},\n");
            } else {
                sb.append("}}");
            }
            ++ik;
        }

        TextFrame.showText("Animation Data", sb.toString(), mf);
    }
    
    
    public static final int[] PORT_MAPPING = {51, 52, 53, 54, 28, 29, 30, 31, 47, 46, 45, 44, 43, 42, 41, 40, 56, 57, 58, 59, 48, 49, 72, 69, 39, 38, 37, 36, 35, 34, 33, 32, 21, 23, 65, 63, 9, 11, 13, 15, 20, 22, 64, 66, 8, 10, 12, 14, 24, 25, 26, 27, 16, 17, 18, 19, 7, 6, 5, 4, 3, 2, 1, 0};
    //public static final int[] PHASE_COMPENSATION = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    public static final int[] PHASE_COMPENSATION = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
}
