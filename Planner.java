//
// Author: Azali Saudi
// Date Created : 30 Dec 2016
// Last Modified: 02 Aug 2021
// Task: The GUI for Robot Path Planning
//

import java.awt.*;
import java.awt.event.*;
import java.applet.*;
import javax.swing.*;
import javax.swing.event.*;
import java.util.*;
import java.io.*;
import javax.imageio.*;
import java.awt.image.*;
import java.util.concurrent.TimeUnit;
import java.awt.geom.Ellipse2D;
import java.util.Timer;
import java.util.TimerTask;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Queue;

public class Planner extends JFrame implements ActionListener {
    //Display parameters
    public static final int Width = 600;
    public static final int Height = 300;

    //Menu Options
    public static final String LOAD_MAP = "Load Map...";
    public static final String RUN_ITER = "Run";
    public static final String RUN_GDS  = "GDS";
    public static final String SAVE_MAP = "Save Map...";

    //GUI Widgets
    public static JLabel label;
    public static JTextField tfMethod;
    public static JTextField tfStartX;
    public static JTextField tfStartY;
    public static JTextField tfGoalX;
    public static JTextField tfGoalY;
    public static JTextField tfW1;
    public static JScrollPane spNote;
    public static JTextArea taNote;
    public static Map canvas;
    public static JMenuBar menu;
    public static JMenu fileMenu;
    public static BufferedImage mapImage;
    public static Timer timer;
    public static Queue<Point> Q;
    public static String fileName;

    //The solver
    public Solver solver;
    public Thread iteratorThread;

    public Planner() {
        setTitle("Planner");
        setLayout(null);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        Container content = getContentPane();

        menu = new JMenuBar();
        fileMenu = new JMenu("File");
        fileMenu.addActionListener(this);
        fileMenu.add(LOAD_MAP).addActionListener(this);
        fileMenu.addSeparator();
        fileMenu.add(RUN_ITER).addActionListener(this);
        fileMenu.add(RUN_GDS ).addActionListener(this);
        fileMenu.addSeparator();
        fileMenu.add(SAVE_MAP).addActionListener(this);
        fileMenu.addSeparator();
        fileMenu.add("Exit").addActionListener((ActionEvent event) -> { System.exit(0); });
        menu.add(fileMenu);

        menu.setBounds(0, 0, Width, 20);
        content.add(menu);

        label = new JLabel("0");
        label.setBounds(10,20, 400,30);
        content.add(label);

        canvas = new Map();
        canvas.setBounds(10, 50, Width-40, Height);
        content.add(canvas);

        tfMethod = new JTextField("SOR");
        tfMethod.setBounds(200,Height+60, 80,25);
        content.add(tfMethod);

        tfStartX = new JTextField("25");
        tfStartX.setBounds(200,Height+85, 39,25);
        content.add(tfStartX);
        tfStartY = new JTextField("246");
        tfStartY.setBounds(241,Height+85, 39,25);
        content.add(tfStartY);
        tfGoalX = new JTextField("148");
        tfGoalX.setBounds(200,Height+110, 39,25);
        content.add(tfGoalX);
        tfGoalY = new JTextField("146");
        tfGoalY.setBounds(241,Height+110, 39,25);
        content.add(tfGoalY);
        tfW1 = new JTextField("1.80");
        tfW1.setBounds(10,Height+60, 50,25);
        content.add(tfW1);

        taNote = new JTextArea("");
        JScrollPane spNote = new JScrollPane(taNote);
        spNote.setBounds(300,Height+60, 270,96);
        content.add(spNote);

        fileName = "case08.png";
        taNote.append(fileName + "\n");
        mapImage = new BufferedImage(1, 1, BufferedImage.TYPE_INT_RGB);

        setSize(Width, Height+200);
        setVisible(true);
    }

    public void init() {
        try {
            mapImage = ImageIO.read(new File(fileName));
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    class Map extends JPanel {
        public Map() {
            addMouseListener(new MyMouse());
        }

        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            int x, y;
            Ellipse2D.Double circle;

            g.drawImage(mapImage, 0, 0, null);

            Graphics2D g2d = (Graphics2D)g;

            x = Integer.parseInt(tfStartX.getText());
            y = Integer.parseInt(tfStartY.getText());
            circle = new Ellipse2D.Double(x-4, y-4, 8.0, 8.0);
            g.setColor(Color.RED);
            g2d.fill(circle);

            x = Integer.parseInt(tfGoalX.getText());
            y = Integer.parseInt(tfGoalY.getText());
            circle = new Ellipse2D.Double(x-4, y-4, 8.0, 8.0);
            g.setColor(Color.GREEN);
            g2d.fill(circle);
        }

        private class MyMouse extends MouseAdapter {
            public void mousePressed(MouseEvent evt) {
                int x = evt.getX();
                int y = evt.getY();
                if(SwingUtilities.isLeftMouseButton(evt)) {
                    tfStartX.setText(Integer.toString(x));
                    tfStartY.setText(Integer.toString(y));
                }
                else
                if(SwingUtilities.isRightMouseButton(evt)) {
                    tfGoalX.setText(Integer.toString(x));
                    tfGoalY.setText(Integer.toString(y));
                }
                repaint();
            }
        }
    }

    public void actionPerformed(ActionEvent evt) {
        String str = evt.getActionCommand();
        if (str.equals(LOAD_MAP)) {
            try {
                JFileChooser chooser = new JFileChooser(new File(".").getCanonicalPath());
                int returnVal = chooser.showOpenDialog(null);
                if(returnVal == JFileChooser.APPROVE_OPTION) {
                    File fout = chooser.getSelectedFile();
                    fileName = fout.getName();
                    mapImage = ImageIO.read(new File(fileName));
                    taNote.append(fileName + "\n");
                    canvas.repaint();
                }
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }
        else if (str.equals(RUN_ITER)) {
            init();   // Load map file
            canvas.repaint();

            iteratorThread = new Thread(new Iterator());
            iteratorThread.start();
        }
        else if (str.equals(RUN_GDS)) {
            int x = Integer.parseInt(tfStartX.getText());
            int y = Integer.parseInt(tfStartY.getText());
            solver.runGDS(mapImage, x, y);
            timer = new Timer();
            timer.schedule(new RemindTask(),
                           0,     // initial delay
                           20);   // subsequent rate in ms
        }
        else if (str.equals(SAVE_MAP)) {
            try {
                JFileChooser chooser = new JFileChooser(new File(".").getCanonicalPath());
                int returnVal = chooser.showSaveDialog(null);
                if(returnVal == JFileChooser.APPROVE_OPTION) {
                    File fout = chooser.getSelectedFile();
                    String saveFilename = fout.getName();
                    int wi = mapImage.getWidth();
                    int hi = mapImage.getHeight();
                    BufferedImage bi = new BufferedImage(wi, hi, BufferedImage.TYPE_INT_RGB);
                    canvas.paint(bi.getGraphics());
                    ImageIO.write(bi, "png", new File(saveFilename));
                }
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void displayLog(int iter, long elaps) {
        // Print out text area
        taNote.append("Iteration: " + Integer.toString(iter) + "\n");
        taNote.append(String.format("Elapsed: %d min, %d sec, %d ms\n",
                                   (elaps/1000) / 60,
                                   (elaps/1000) % 60,
                                   (elaps%60000) % 1000));
    }

    class RemindTask extends TimerTask {
        Point pt = new Point();
        Graphics2D g2D;

        RemindTask() {
            g2D = mapImage.createGraphics();
            g2D.setColor(Color.BLACK);
        }

        public void run() {
            pt = solver.path.remove();
            g2D.fill(new Ellipse2D.Double(pt.x, pt.y, 1.5, 1.5));
            canvas.repaint();
            if(solver.path.isEmpty()) timer.cancel();
        }
    }

    class Iterator implements Runnable {
        public void run() {
            int gx = Integer.parseInt(tfGoalX.getText());
            int gy = Integer.parseInt(tfGoalY.getText());
            double w = Float.parseFloat(tfW1.getText());
            int iteration = 0;
            boolean converge = false;
            long startTime = System.nanoTime();

            solver = new Solver(mapImage, gx, gy);
            if (tfMethod.getText().toUpperCase().equals("SOR")) {
                while(!converge) {
                    solver.doSOR(w);
                    label.setText(String.format("%d", ++iteration));
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                taNote.append(String.format(">>> SOR, w=%.2f\n", w));
            }

            long stopTime = System.nanoTime();
            long elapsed = stopTime - startTime;
            elapsed = TimeUnit.NANOSECONDS.toMillis(elapsed); // Total elapsed in ms
            displayLog(iteration, elapsed);
        }
    }

    public static void main(String[] args) {
        Planner a = new Planner();
        a.init();
    }
}

