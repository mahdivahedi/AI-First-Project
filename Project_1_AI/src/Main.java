import java.awt.*;
import java.io.FileNotFoundException;


public class Main {

    public static void main(String[] args) throws FileNotFoundException {
        PathFinder p = new PathFinder("/home/meyti/Documents/AI/Projects/Project_1_AI/src/test1.txt");
        var path = p.aStar(new Point(2, 2));
        System.out.println("********************************");
        path.forEach(System.out::println);
    }

}
