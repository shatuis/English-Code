import java.util.Scanner;


public class triangle {
	
	  
	   public static int hypotenuse(int a, int b){
	      
		   if(a < b)
	      {
		  double hypot = Math.hypot(a, b);
	      if (hypot % 1 == 0 && b < hypot){
	         return (int)hypot;
	      }
	      }
	      return -1;
	   }
	 
	   public static void iterate(int maxVal){
	      for (int i = 1; i<maxVal+1; i++){
	         for (int j = 1; j<maxVal+1; j++){
	            int hypo = hypotenuse(i,j);
	            if(hypo > -1){
	               System.out.println("[ a="+i+" , b="+j+" , c="+hypo+" ]);
	            }
	         }
	      }
	   }
	 
	   public static void main(String[] args) {
		   Scanner keyboard = new Scanner (System.in);
		   int k = keyboard.nextInt();
		   triangle triple = new triangle();
		   triple.iterate(k);
	   }
}
