import java.util.Random;
import java.util.Scanner;

public class Main
{

    private Main() {}
    
    public static void main(String[] args) 
    {

        boolean run = true;

        Scanner keys = new Scanner(System.in);
        
        String message = "prediction is incorrect";

        int counter = 0;

        while (run)
        {
             System.out.println("1 or 2");

            int prediction = keys.nextInt();

            if (coinFlip() == prediction)  
            {
                message = "prediction is correct";
            }
            else                           
            {
                message = "you a dummy";
                counter ++;
            }

            System.out.println(message);

            System.out.println();

            if (counter == 3)
            {
                run = false;
            }
        }

    }

    public static int coinFlip()
    {
        Random gen = new Random();
        return gen.nextInt(1,3);
    }

}



