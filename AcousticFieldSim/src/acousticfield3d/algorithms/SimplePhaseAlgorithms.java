package acousticfield3d.algorithms;


import acousticfield3d.math.M;
import acousticfield3d.math.Vector2f;
import acousticfield3d.math.Vector3f;
import acousticfield3d.scene.Scene;
import acousticfield3d.simulation.Transducer;
import java.util.ArrayList;
import java.util.List;


 class TransItem {
    
    public String name;
    public float amplitude;
    public float phase;
    public int orderNumber;
    public int driverPinNumber;
    
 //   public TransItem(String name, float amplitude, float phase, 
 //                int orderNumber, int driverPinNumber) 
 //  {
 //   this.name = name;
 //   this.amplitude = amplitude;
 //   this.phase = phase;
 //   this.orderNumber = orderNumber;
  //  this.driverPinNumber = driverPinNumber;
    
  // }
    
    public static void printTable(TransItem[][] table) 
    {
         System.out.format("cell of the able: \n");
         System.out.format("transducer amplitude phase orderNumber driverPinNumber\n");
                         
        for (int i =0; i < 8; i++) {
            for (int j=0; j < 8; j++) {
                // System.out.format("%4s%4s%4s%3s%3s", table[i][j].name, 
                 System.out.format("%4s%4s%4s%3s", table[i][j].name,
                          table[i][j].amplitude, table[i][j].phase, 
                          table[i][j].orderNumber);
                 // , table[i][j].driverPinNumber);
                 System.out.format("|");
            }
            System.out.format("\n");
        }
          
    }//printTable
    
 } // TransItem
/**
 *
 * @author Asier
 */
public class SimplePhaseAlgorithms {
  
    public static void focus(final Transducer t, final Vector3f target, float speedOfSound){
            final float distance = target.distance( t.getTransform().getTranslation() );
            final float waveLength = speedOfSound / t.getFrequency();
            final float targetPhase = ( 1.0f - M.decPart(distance / waveLength)  ) * 2.0f * M.PI;
            //  M.decPart (a) = return a - ((int)a);
            //  Derivation of targetPhase: (L_cf - L_tf) * ( 2pi/lambda ) = phaseDiff
            // 
            t.setPhase(targetPhase / M.PI ); // targetPhase is in radian, 
              // The unit of the phase used in this system is pi. So phase 2 means 2 * pi 
               
                                              
    }
    
    public static void focus(final List<Transducer> trans, final Vector3f target, float speedOfSound){
        for(Transducer t : trans){
            final float distance = target.distance( t.getTransform().getTranslation() );
            final float waveLength = speedOfSound / t.getFrequency();
            final float targetPhase = (1.0f - M.decPart(distance / waveLength) ) * 2.0f * M.PI;
            t.setPhase(targetPhase / M.PI );
        }
        
                         
 // MJ, 2020/5/18: debugging
 // Print the informaton about the transducer array
       
 //  public String name;
 //   public float amplitude; //from 0 to 1
 //   public float phase; //in radians but divided by PI. That is, a phase of 2 means 2PI radians    
  //  private int orderNumber;
  //  private int driverPinNumber; //in the driver board
   
        TransItem[][] table; // MJ: for debugging, 2020/05/18
        table = new TransItem[8][8]; // MJ: for debugging,  2020/05/18
        
        //ArrayList<Transducer> transducers = simulation.getTransducers() ;
       
       // if (transducers.isEmpty() ) {
       //     return;
       // }
       // Create objects for object array table
       for (int k =0; k < 8; k++) {
            for (int l=0; l < 8; l++) {
                table[k][l] = new TransItem();
                 
            }
       }
       
       //print  the transducers' states from the calculation of the field
       // The amplitudes are all 1?; Only the phases are calculated?
       
        for (Transducer t :  trans) {
                    String name = t.name;
                    float amplitude = t.amplitude;
                    float phase =  t.phase; //in radians but divided by PI. That is, a phase of 2 means 2PI radians    
                    int orderNumber = t.getOrderNumber();
                    int driverPinNumber = t.getDriverPinNumber(); //in the driver board
                  
                    // get the grid coordinate in the array
                    int i = orderNumber / 8;
                    int j = orderNumber % 8;
                    
                     table[i][j].name = name ;
                     table[i][j].amplitude = amplitude ;
                     table[i][j].phase = phase;
                     table[i][j].orderNumber = orderNumber ;
                     table[i][j].driverPinNumber = driverPinNumber;
                     
                                
                                      
            }//   for (Transducer t :  transducers)
               
            TransItem.printTable( table );  
           //// the end of debugger code
        
    }//focus
    
    public static void addTwinSignature(final List<Transducer> trans, final float angle){
        Vector3f min = new Vector3f(), max = new Vector3f();
        Scene.calcBoundaries(trans, min, max);
        final Vector3f size = max.subtract( min );
        final Vector3f center = max.add(min).divideLocal( 2 );
        
        for(Transducer t : trans){
            final Vector3f pos = t.getTransform().getTranslation();
            final Vector3f npos3 = pos.subtract( center ).divideLocal( size );
            final Vector2f p = new Vector2f( npos3.x, npos3.z);
            
            float value = 0;
            value = (p.getAngle() + angle) / M.PI % 2.0f;
            if (value >= 0.0f && value <= 1.0f) { value = 0.0f;}
            else { value = 1.0f; }
            
            t.phase += value;
        }
    }
    
    public static void addVortexSignature(final List<Transducer> trans, final float m){
        Vector3f min = new Vector3f(), max = new Vector3f();
        Scene.calcBoundaries(trans, min, max);
        final Vector3f size = max.subtract( min );
        final Vector3f center = max.add(min).divideLocal( 2 );
        
        for(Transducer t : trans){
            final Vector3f pos = t.getTransform().getTranslation();
            final Vector3f npos3 = pos.subtract( center ).divideLocal( size );
            final Vector2f p = new Vector2f( npos3.x, npos3.z);
            
            float value = 0;
            value = (p.getAngle() * m) / M.PI % 2.0f;
            
            t.phase += value ;
        }
    }
            
}
