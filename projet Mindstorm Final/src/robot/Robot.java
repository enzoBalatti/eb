package robot;


import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.utility.Delay;
import lejos.hardware.motor.*;
import lejos.hardware.port.*;
import lejos.hardware.sensor.*;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.chassis.*;
import java.io.*;  
import java.util.*;

/**
 * @author Balatti Enzo, Bellatif Mamoune, Bergman Clement
 *	
 */

/**
 * @author enzob
 *
 */
public class Robot {
	EV3MediumRegulatedMotor mG;// ce moteur correspond a la roue gauche
	EV3MediumRegulatedMotor mD;// ce moteur correspond a la roue droite
	EV3MediumRegulatedMotor mP;// ce moteur correspond aux moteurs des pinces
	
	Wheel rG; //Roue gauche
	Wheel rD; //Roue droite
	Chassis chassis; 

	EV3ColorSensor col; // capteur couleur
	SampleProvider sampleCol;

	EV3TouchSensor pression; // capteur pression
	SampleProvider samplePress;
	float [] tabPress;

	EV3UltrasonicSensor ultrason;// capteur ultrasons
	SampleProvider sampleUS;
	float [] usSample;
	float distanceValue;
	
	FileWriter fichierCouleurs;//fichier de lecture 
	BufferedReader bufCol;//fichier d'ecriture

	double angleCrt;
	
	String enBut="";
	// mur VS salle
	String coteDepart="";
	// gauche VS milieu VS droite
	int angleDepart=0; //angle utilisé dans la méthode marquerPremierPalet pour savoir si tourner à gauche ou à droite

	/**
	 * 
	 */
	public Robot() {
		mG=new EV3MediumRegulatedMotor(LocalEV3.get().getPort("A"));
		mD=new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
		
		rG= WheeledChassis.modelWheel(mG,56).offset(-61.5);
		rD= WheeledChassis.modelWheel(mD,56).offset(61.5);
		chassis=new WheeledChassis(new Wheel[] {rG,rD},WheeledChassis.TYPE_DIFFERENTIAL);//chassis permet de synchroniser les deux roues
		chassis.setSpeed(360.0, 345.0);

		mP=new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

		col=new EV3ColorSensor(LocalEV3.get().getPort("S4"));
		col.setFloodlight(Color.WHITE);

		sampleCol=new MeanFilter(col.getRGBMode(), 1);

		pression=new EV3TouchSensor(LocalEV3.get().getPort("S2"));
		samplePress= pression.getTouchMode();
		tabPress=new float[samplePress.sampleSize()];

		ultrason=new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		sampleUS=ultrason.getDistanceMode();
		usSample= new float[sampleUS.sampleSize()];
		sampleUS.fetchSample(usSample, 0);
		distanceValue = usSample[0];
		
		angleCrt=0;//l'angle courant est initialise a zero, il sera modifié a chaque fois que le robot tourne. lorsque se valeur est 0, le robot se situe face a la ligne d'en-but adverse
	}

	
	/**
	 * @return la taille de l'échantillon du capteur de couleur
	 */
	public int getSample() {
		return sampleCol.sampleSize();
	}

	/**
	 * @param v1 un tableau de valeur des composantes rouges, vertes et bleues d'une couleur 
	 * @param v2 un tableau de valeur des composantes rouges, vertes et bleues d'une autre couleur
	 * @return retourne la somme des écarts entre les différentes composantes rouges, vertes et bleue du tableau v1 et v2 
	 */
	public static double scalaire(float[] v1, float[] v2) {
		return Math.sqrt (Math.pow(v1[0] - v2[0], 2.0) +
				Math.pow(v1[1] - v2[1], 2.0) +
				Math.pow(v1[2] - v2[2], 2.0));
	}


	/**Permet de stocker les valeurs des composantes rouges, vertes et bleues de chaque couleur présente sur le plateau 
	 * sauf la principale, le gris. 
	 * Ecrit dans le fichier couleur.txt les valeurs RVB.
	 * @throws IOException
	 */
	public void calibrageCol() throws IOException {

		try {
			fichierCouleurs = new FileWriter("/home/lejos/programs/couleurs.txt"); //on crée un nouveau fichier d'ecriture.
		}catch(Exception e) {System.out.println(e);}

		System.out.println("Press enter to calibrate blue...");
		Button.waitForAnyPress();
		col.setFloodlight(Color.WHITE); // on utilise un faisceau blanc pour detecter les couleurs
		float[] blue = new float[sampleCol.sampleSize()]; // creation du tableau des composantes RVB du capteur
		sampleCol.fetchSample(blue, 0); // les valeurs detectées sont rangées dans le tableau col, à partir de l'indice 0
		String bl= (String)""+blue[0]+"/"+blue[1]+"/"+blue[2];

		try {
			fichierCouleurs.write(bl); //on écrit dans le fichier les différentes valeurs pour la couleur bleue
			fichierCouleurs.write("\n");//on saute une ligne 

		}catch(Exception e) {System.out.println(e);}

		Button.waitForAnyPress();

		System.out.println("Press enter to calibrate red...");
		Button.waitForAnyPress();
		col.setFloodlight(Color.WHITE);
		float[] red = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(red, 0);
		String re= (String) ""+red[0]+"/"+red[1]+"/"+red[2];

		try {
			fichierCouleurs.write(re);
			fichierCouleurs.write("\n");
		}catch(Exception e) {System.out.println(e);}
		Button.waitForAnyPress();


		System.out.println("Press enter to calibrate green...");
		Button.waitForAnyPress();
		col.setFloodlight(Color.WHITE);
		float[] green = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(green, 0);
		String gr= (String) ""+green[0]+"/"+green[1]+"/"+green[2];;

		try {
			fichierCouleurs.write(gr);
			fichierCouleurs.write("\n");
		}catch(Exception e) {System.out.println(e);}
		Button.waitForAnyPress();

		System.out.println("Press enter to calibrate black...");
		Button.waitForAnyPress();
		col.setFloodlight(Color.WHITE);
		float[] black = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(black, 0);
		String bla= (String) ""+black[0]+"/"+black[1]+"/"+black[2];

		try {
			fichierCouleurs.write(bla);
			fichierCouleurs.write("\n");
		}catch(Exception e) {System.out.println(e);}
		Button.waitForAnyPress();

		System.out.println("Press enter to calibrate yellow...");
		Button.waitForAnyPress();
		float[] yellow = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(yellow, 0);
		String ye= (String) ""+yellow[0]+"/"+yellow[1]+"/"+yellow[2];

		try {
			fichierCouleurs.write(ye);
			fichierCouleurs.write("\n");
		}catch(Exception e) {System.out.println(e);}
		Button.waitForAnyPress();

		System.out.println("Press enter to calibrate white...");
		Button.waitForAnyPress();
		float[] white = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(white, 0);
		String wh= (String) ""+white[0]+"/"+white[1]+"/"+white[2];

		try {
			fichierCouleurs.write(wh);
			fichierCouleurs.write("\n");
		}catch(Exception e) {System.out.println(e);}
		fichierCouleurs.close();  //On ferme le flux d'écriture 
		Button.waitForAnyPress();




	}

	/**
	 * Permet de savoir quelle est la couleur detectée par le capteur de couleur. Cette méthode compare la couleur detectée avec toutes 
	 * celle qu'il a calibré au préalable. Grace à l'appel de la methode sclaire, la couleur ayant le plus petit écart avec 
	 * celle detectée est retournée
	 * @return la chaîne de caractère de la couleur identifiée par le capteur de couleurs
	 * @throws IOException
	 */
	public String getCouleur() throws IOException{
		float[] sample = new float[sampleCol.sampleSize()];
		//System.out.println("\nPress enter to detect a color...");
		//Button.waitForAnyPress();
		sampleCol.fetchSample(sample, 0);
		double minscal = Double.MAX_VALUE;
		String color = "";

		//extraction fichier
		bufCol=  new BufferedReader(new FileReader("/home/lejos/programs/couleurs.txt")); //On va lire le fichier via un flux de lecture 
		float[] blue= new float[sampleCol.sampleSize()]; // on crée un tableau pour les valeurs RVB de chaque couleur
		float[] red= new float[sampleCol.sampleSize()];
		float[] green= new float[sampleCol.sampleSize()];
		float[] black= new float[sampleCol.sampleSize()];
		float[] yellow= new float[sampleCol.sampleSize()];
		float[] white= new float[sampleCol.sampleSize()];
		String[] split=new String[sampleCol.sampleSize()];
		String s= bufCol.readLine(); //on extrait la ligne lue dans la chaîne s
		//System.out.println(s);
		split=s.split("/"); //avec la méthode split, on decoupe la chaîne dans le tableau, le séparateur étant "/"
		blue[0]=new Float(split[0]); // le tableau de valeurs RVB de chaque couleur est rempli par celles de split
		blue[1]=new Float(split[1]);
		blue[2]=new Float(split[2]);
		split=bufCol.readLine().split("/"); // une fois le tableau rempli, split stocke les valeurs de la ligne suivante
		red[0]=new Float(split[0]); 
		red[1]=new Float(split[1]);
		red[2]=new Float(split[2]);
		split=bufCol.readLine().split("/");
		green[0]=new Float(split[0]);
		green[1]=new Float(split[1]);
		green[2]=new Float(split[2]);
		split=bufCol.readLine().split("/");
		black[0]=new Float(split[0]);
		black[1]=new Float(split[1]);
		black[2]=new Float(split[2]);
		split=bufCol.readLine().split("/");
		yellow[0]=new Float(split[0]);
		yellow[1]=new Float(split[1]);
		yellow[2]=new Float(split[2]);
		split=bufCol.readLine().split("/");
		white[0]=new Float(split[0]);
		white[1]=new Float(split[1]);
		white[2]=new Float(split[2]);
		bufCol.close();

		double scalaire = scalaire(sample, white);
		//System.out.println(scalaire);
		if (scalaire < minscal) {
			minscal = scalaire;
			color = "white";
		}
		scalaire = scalaire(sample, blue);
		//System.out.println(scalaire);
		if (scalaire < minscal) {
			minscal = scalaire;
			color = "blue";
		}

		scalaire = scalaire(sample, red);
		//System.out.println(scalaire);
		if (scalaire < minscal) {
			minscal = scalaire;
			color = "red";
		}

		scalaire = scalaire(sample, green);
		//System.out.println(scalaire);
		if (scalaire < minscal) {
			minscal = scalaire;
			color = "green";
		}

		scalaire = scalaire(sample, black);
		//System.out.println(scalaire);
		if (scalaire < minscal) {
			minscal = scalaire;
			color = "black";
		}

		scalaire = scalaire(sample, yellow);
		//System.out.println(scalaire);
		if (scalaire < minscal) {
			minscal = scalaire;
			color = "yellow";
		}


		//System.out.println(color);
		return color;

	}


	/**
	 * permet de savoir si un palet est entre les pinces.
	 *  Si le capteur de pression est activé, alors il y a un palet dans les pinces
	 * @return boolean
	 * 
	 */
	public boolean paletDansPince() {
		samplePress.fetchSample(tabPress, 0);
		return(tabPress[0]==1);//1 ==> pression activée
	}

	
	/**
	 * Lorsqu’un palet est détecté, cette méthode est appelée, le robot ouvre alors ses pinces, s’approche du palet et le saisit. 
	 * Si aucun palet n’est saisi ou si le robot traverse une ligne blanche, le robot ferme ses pinces et appelle la méthode 
	 * ChercherObstaclePlusProche.
	 * @throws IOException
	 */
	public void ramasserPalet() throws IOException{
		mP.forward(); //ouverture des pinces

		while(!paletDansPince()) {
			if(mP.getPosition()>700) {// tant que le palet n'est pas dans les pinces et que l'ouverture des pinces est inferieure a 700, on ouvre les pinces
				break;
			}
		}
		mP.stop();// on stoppe le moteur des pinces une fois la position des pinces a 700
		chassis.travel(350);
		while(!paletDansPince()) {// tant que pas de palet dans les pinces et que la distance n'a pas ete parcourue on reste dans le while
			if(getCouleur().equals("white") || getCouleur().equals("red") || getCouleur().equals("yellow")) {// le robot doit rester entre la ligne jaune et la ligne rouge pour éviter qu'il aille contre un mur, il ne doit pas non plus aller dans les zones d'en-but
				mP.backward();
				while(mP.getPosition()>20) { 
				}
				mP.stop();
				chassis.travel(-300);
				while(chassis.isMoving()) {}
				chercherObstaclePlusProche();// si le robot est allé la ou il ne devait pas on recule et on cherche l'obstacle le plus proche
			}
			if(!chassis.isMoving()) {// si le robot s'arrete d'avancer, c'est qu'il n'a pas attrapé de palet, il doit alors fermer ses pinces et chercher un autre palet
				mP.backward();
				while(mP.getPosition()>20) {
				}
				mP.stop();
				chassis.travel(-550);
				while(chassis.isMoving());
				chercherObstaclePlusProche();
			}
		}
		chassis.stop();// ici le robot a attrapé un palet, il s'arrete alors et ferme ses pinces
		mP.backward();
		while(mP.getPosition()>20) {
		}
		mP.stop();
	}


	/**
	 * permet au robot d'effectuer un virage d'un angle de rotation choisi en degrés, il modifie aussi la variable angleCrt
	 * @param rotation 
	 */
	public void virage(double rotation){
		chassis.rotate(rotation);
		angleCrt+=rotation;
		if(angleCrt>360)
			angleCrt-=360;
		else if (angleCrt<-360)
			angleCrt+=360;
	}



	/**
	 * permet au robot d'avancer d'une distance choisie en millimetres
	 * @param distance 
	 */
	public void avancerDe(double distance) {
		chassis.travel(distance);
	}

	/**
	 * Cette méthode est une méthode static, qui prends en paramètre une liste, 
	 * et qui renvoie l’index de la plus petite valeur supérieure a 340 de cette liste.
	 * @param tab
	 * @return index de la valeur minimale superierue a 340
	 */
	public static int getMin(List<Double> tab) {
		Double min = Double.MAX_VALUE;
		int index=-1;
		for (int i=0;i<tab.size();i++) {
			if(tab.get(i)<min && tab.get(i) > 340 ) {
				min=tab.get(i);
				index=i;
			}
		}
		return index;
	}


	/**
	 *Lors de l’appel de cette méthode, le robot fait un tour sur lui même et toutes les millisecondes,
	 *il stocke la valeur de la distance de l’objet qu’il a en face de lui dans une liste.
	 *A la fin du tour, grâce à la fonction getMin, le robot récupère la distance la plus courte qu’il a identifié et 
	 *s’oriente vers l’objet correspondant a cette distance. Puis il détermine en s’avançant vers cet objet
	 *si il correspond à un palet, à un robot ou à un mur. Si c’est un palet, un appel a la fonction ramasserPalet, 
	 *puis à la méthode marquerPalet sont effectués. Sinon le Robot recule, et un appel a la méthode chercherObstaclePlusProche est effectué. 
	 *Cette méthode est donc récursive. 
	 * @throws IOException
	 */

	public void chercherObstaclePlusProche() throws IOException{
		chassis.setAngularSpeed(60);//on donne une vitesse de rotation faible au robot pour qu'il recupere un grand nombre de valeur
		ArrayList<Double> tabDisCrt =new ArrayList<Double>();
		virage(360);
		while(chassis.isMoving()) {
			sampleUS.fetchSample(usSample, 0); // attribution valeur capteur US à case 0 du tableau usSample
			double distance = usSample[0];
			tabDisCrt.add(distance*1000);// on stocke les distances dans une liste
			Delay.msDelay(1);//on passe dans la boucle toutes les millisecondes
		}
		int index = getMin(tabDisCrt);//on recupere la valeur de l'index correspondant a la distance de l'objet le plus proche pouvant etre un palet
		double angleMin = (double)(360.0/(double)tabDisCrt.size())*(double)index;//on determine l'angle correspondant a l'objet identifie comme le plus proche
		double distanceMin = tabDisCrt.get(index);// on recupere la distance existante entre le robot et l'objet le plus proche
		chassis.setAngularSpeed(295.0);// On remet la vitesse de rotation a sa valeur initiale
		if (angleMin>180)
			angleMin=-(angleMin-180);
		virage(angleMin);//on s'oriente vers l'objet identifie
		while(chassis.isMoving()) {};
		chassis.travel(distanceMin-150);//on avance jusqu'a 15 cm avant l'objet 
		while(chassis.isMoving()) {
			if(getCouleur()=="white") {// si une ligne blanche est detectee on fait demi tour et on cherche un autre palet
				chassis.stop();
				if((angleCrt<90 || angleCrt> 270)) {//dans ce cas le robot est face a la ligne d'en-but adverse
					virage(180-angleCrt);//on calcule l'angle pour que le robot s'oriente face a sa propre ligne d'en-but
					while(chassis.isMoving()) {}
					chassis.travel(400);
					chercherObstaclePlusProche();
				}
				else {//dans ce cas le robot est face a sa ligne d'en-but propre
					virage(angleCrt-180);
					while(chassis.isMoving()) {}
					chassis.travel(400);
					chercherObstaclePlusProche();
				}
			} 
			else if(getCouleur()=="yellow" || getCouleur()=="red") { //si on passe une ligne rouge ou jaune on recule et on cherche un autre palet
				chassis.travel(-400);
				while(chassis.isMoving()) {}
				chercherObstaclePlusProche();
			}
		}
		//on a avance jusqua 15 cm avant l'obstacle, si il n'y est plus c'est un palet (ou un robot) il doit alors le saisir. si c'etait un robot, ou que le palet n'y est plus la fonction chercherObstacle le plus proche sera appelee
		sampleUS.fetchSample(usSample, 0); 
		double distance = usSample[0];
		if (distance*1000>170) {//si la distance par rapport a l'objet initialement identifiée est superieure a 17 cm, alors c'etait un palet ou un robot, une erreur de 2 cm est anticipée sur les mesures du robot
			ramasserPalet();//on ramasse et on marque le palet
			marquerPalet();
		}
		else {// si la distance est superieure a 17 cm on recule et on rappelle la methode chercherObstaclePlusProche
			chassis.travel(-200);
			while(chassis.isMoving()) {};
			chercherObstaclePlusProche();
		}
	}

	/**
	 * Lorsque cette méthode est appelée, le robot ouvre ses pinces, recule, tourne de 180 degrés et ferme ses pinces.
	 * @throws IOException
	 */
	public void deposerPalet() throws IOException {
		mP.forward(); //ouverture des pinces
		while(mP.getPosition()<500) {}
		mP.stop();
		chassis.travel(-100);
		while(chassis.isMoving()) {}
		virage(180);
		while(chassis.isMoving()) {}
		mP.backward();
		while(mP.getPosition()>20) {}
		mP.stop();
	}


	/**
	 * Cette méthode sera appelée en premier lors de matchs, elle fait saisir au robot le palet situé en face de lui lors 
	 * des coups d’envois de matchs, et lui fait marquer ce palet dans la zone d’en-but adverse.
	 * @throws IOException
	 */
	public void marquerPremierPalet() throws IOException {
		mP.forward(); //ouverture des pinces
		chassis.travel(700);
		while(!paletDansPince() ) {
			if(mP.getPosition()>500) {// on ouvre les pinces jusqu'a ce qu'elles soient a la position 500, et tant que le palet n'est pas dans les pinces
				break;
			}
		}
		mP.stop();//on arrete les moteurs des pinces une fois celles ci ouvertes
		while(!paletDansPince()) {}//on continue d'avancer dans que le palet n'est pas dans les pinces
		chassis.stop();//le robot arrete d'avancer une fois le palet saisi

		mP.backward();
		while(mP.getPosition()>10) {// on ferme les pinces jusqu'a ce qu'elles atteignent le position 10
		}
		mP.stop();
		virage(angleDepart); // on tourne de l'angle défini dans la methode avantMatch(), on avance de 40 cm puis on se remet en face de l'en-but adverse pour eviter de rentrer en collision avec d'autres palets
		while(chassis.isMoving()) {}
		chassis.travel(400);
		while(chassis.isMoving()) {}
		virage(-angleDepart);
		while(chassis.isMoving()) {}
		chassis.travel(2000);// a cet instant le robot est a moins de deux metres de la ligne d'en-but adverse, il avance donc jusqu'a detecter la ligne blanche, mais finira par s'arreter si il ne la detecte pas.
		while(chassis.isMoving()) {// on avance jusqu'a la ligne d'en-but adverse
			if(getCouleur()=="white") {
				chassis.stop();
			}
		}
		try{
			deposerPalet();
		}catch(Exception e) {System.out.println(e);}
		chassis.travel(800);//on se replace dans le terrain
		while(chassis.isMoving()) {}
		chercherObstaclePlusProche();// on cherche l'obstacle le plus proche
	}

	/**Lorsque cette méthode est appelée, 
	 * le robot s’oriente vers la zone d’en-but adverse et avance jusqu’à celle ci pour y déposer le palet.
	 * @throws IOException
	 */
	public void marquerPalet() throws IOException{
		if(angleCrt>0 && angleCrt<180) {// orientation vers ligne de but adverse
			virage(-angleCrt);
			while(chassis.isMoving()) {}
		}
		else {
			virage(360-angleCrt);
			while(chassis.isMoving()) {}
		}
		chassis.travel(3000);//avancer de trois metres permet que le robot finisse par s'arreter meme si il ne detecte pas la ligne blanche
		int a=0;
		while(chassis.isMoving()) {//on avance jusqu'a la ligne adverse
			if(getCouleur()=="red" ) { //si on croise une ligne jaune ou rouge on s'arrete et on s'oriente vers la ligne d'en-but adverse
				if(enBut=="mur") {
					chassis.stop();
					virage(-90);
					while(chassis.isMoving()) {}
				}
				else {
					chassis.stop();
					virage(90);
					while(chassis.isMoving()) {}
				}
				a=1;// si on croise une ligne rouge ou jaune la variable a prends la valeur 1 et on sort du while car le chassis s'est arrete
			} 
			else if(getCouleur()=="yellow") {
				if(enBut=="mur") {
					chassis.stop();
					virage(90);
					while(chassis.isMoving()) {}
				}else {
					chassis.stop();
					virage(-90);
					while(chassis.isMoving()) {}
				}
				a=1;
			}
			else if(getCouleur()=="white") {
				chassis.stop();
			}
		}
		if (a==1) {// si a=1, on avance vers la ligne d'en-but adverse
			chassis.travel(3000);
			while(chassis.isMoving()) {
				if(getCouleur()=="white") {
					chassis.stop();
				}
			}
		}
		try{
			deposerPalet();
		}catch(Exception e) {System.out.println(e);}
		chassis.travel(500);// on se replace dans le terrain et on cherche un nouvel obstacle
		while(chassis.isMoving());
		chercherObstaclePlusProche();
	}

	/** Lors de l’appel de cette méthode, le robot est censé s’orienter en face du mur le plus proche.
	 * 
	 */
	public void faceAuMur() {
		
		chassis.setAngularSpeed(60);
		ArrayList<Double> OM =new ArrayList<Double>();
		virage(180);
		while(chassis.isMoving()) {
			sampleUS.fetchSample(usSample, 0); // attribution valeur capteur US à case 0 du tableau usSample
			double distance = usSample[0];
			//System.out.print("distance: "+ distance);
			OM.add(distance*1000);
			//i++;
			Delay.msDelay(1); 	
		}
		int index = getMin(OM);
		//Button.waitForAnyPress();
		double angleMin = (double)(360.0/(double)OM.size())*(double)index;
		//double distanceMin = OM.get(index);
		//System.out.println(" index ="+index +" angle="+angleMin);
		Button.waitForAnyPress();
		//chassis.setAngularSpeed(chassis.getMaxAngularSpeed());// changer valeur ici
		virage(-180+angleMin);
		while(chassis.isMoving()) {}
		OM.clear();
	}


	/**Lors de l’appel de cette méthode le robot doit avancer et identifier les deux premières lignes de couleurs qu’il croise.
	 *  A la fin de cette méthode, la méthode comparaison est appelée avec en paramètre les deux couleurs identifiées
	 * @throws IOException
	 */
	public void replacement() throws IOException{
		String col1="", col2="";
		double distanceSuivante=0;
		//System.out.println("j'y suis");
		Button.waitForAnyPress();
		chassis.travel(600); // on avance pour detecter la couleur de la première ligne (max 600 car taille max d'un côté entre lignes)
		while(chassis.isMoving()) {
			if(getCouleur()!="white") { // si couleur ligne non blanche, on l'enregistre 
				chassis.stop();
				col1=getCouleur(); 
				distanceSuivante=600;// on prévoir une distance jusqu'à la prhcaine ligne de 600
			} else {
				chassis.stop(); // on s'arrête car au niveau de l'enbut
				col1="white";
				distanceSuivante=-600; // marche arrière car besoin d'aller dans le terrain et non dans l'enbut
			}
		}
		chassis.travel(distanceSuivante);  // on avance ou recule pour detecter la couleur de la deuxième ligne
		while(chassis.isMoving()) {
			if(getCouleur()!="white") {// si couleur ligne non blanche, on l'enregistre, 
				chassis.stop();
				col2=getCouleur(); // on arrête et on passe à la comparaison
			} else { //que faire si la couleur de la seconde ligne est blanche ??
				chassis.stop(); // on s'arrête car au niveau de l'enbut
				col2="white"; // on arrête et on passe à la comparaison
			}
		}
		comparaison(col1,col2);
	}

	/**cette méthode determine ou se situe le robot en fonction des deux couleurs identifiées dans la méthode replacement,
	 * puis s’oriente vers la zone d’en-but adverse.
	 * @param couleur1
	 * @param couleur2
	 * @throws IOException
	 */
	public void comparaison(String couleur1, String couleur2)  throws IOException{
		if((couleur1=="yellow" && couleur2=="black") || (couleur1=="black" && couleur2=="red")) {
			if(enBut=="salle") {
				virage(90); //besoin de tourner à 90 degrés vers la droite
			}else {
				virage(-90); //besoin de tourner à 90 degrés vers la gauche
			}
		} else if((couleur1=="red" && couleur2=="black") || (couleur1=="black" && couleur2=="yellow")) {
			if(enBut=="salle") {
				virage(-90); //besoin de tourner à 90 degrés vers la gauche
			}else {
				virage(90); //besoin de tourner à 90 degrés vers la droite
			}
		}else if((couleur1=="green" && couleur2=="black") || (couleur1=="black" && couleur2=="blue")) {
			if(enBut=="salle") {
				//on est aligné dans le bon sens
			}else {
				virage(180); //dans sens opposé, faire demi tour pour être aligné
			}
		}else if((couleur1=="black" && couleur2=="green") || (couleur1=="blue" && couleur2=="black")) {
			if(enBut=="salle") {
				virage(180); //dans sens opposé, faire demi tour pour être aligné
			}else {
				//on est aligné dans le bon sens

			}
		}else if((couleur1=="white" && couleur2=="green") || (couleur1=="green" && couleur2=="white")) {
			if(enBut=="salle") {
				virage(180); //dans sens opposé, faire demi tour pour être aligné
			}else {
				//on est aligné dans le bon sens
			}
		}else if((couleur1=="white" && couleur2=="blue") || (couleur1=="blue" && couleur2=="white")) {
			if(enBut=="salle") {
				//on est aligné dans le bon sens
			}else {
				virage(180);//dans sens opposé, faire demi tour pour être aligné
			}
		} else {// la combinaison de couleur n'est pas spécifiée, le robot doit être de biais et a detectée une couleur adjaçente
			//on se décale de 45 degré et on recommence le replacement
			virage(45);
			faceAuMur();
			replacement();
		}
		angleCrt=0;
	}

	/**cette méthode est appelée lorsque le robot est desorienté
	 * @throws IOException
	 */
	public void procedureDeReplacement()throws IOException {
		faceAuMur();
		virage(180);
		replacement();
		//System.out.println("fin");
		Button.waitForAnyPress();
		chercherObstaclePlusProche();

	}

	/**cette méthode est appelée avant le début d’un match, elle permet d’indiquer au robot ou il va 
	 * commencer pour que la méthode marquerPremierPalet soit correctement executée. 
	 * 
	 */
	public void avantMatch() {
		System.out.println("d'où partez vous ? mur(haut)VS salle(bas)");
		Button.waitForAnyPress();
		if(Button.getButtons()==1)
			enBut="salle"; // On est côté mur, on appuie sur UP, donc l'enBut est côté salle
		enBut="mur"; // On est côté salle, on appuie sure DOWN donc l'enBut est côté mur

		System.out.println("gauche VS milieu VS droite");
		Button.waitForAnyPress();
		if(Button.getButtons()==16)
			coteDepart="gauche";
		else if(Button.getButtons()==2)
			coteDepart="milieu";
		else 
			coteDepart="droite";

		if(coteDepart=="gauche")
			angleDepart=35;
		else
			angleDepart=-35;

		System.out.println("appuie pour lancer le match");
		Button.waitForAnyPress();
	}

	/**
	 * @param args
	 * @throws IOException
	 */
	public static void main(String[] args) throws IOException {
		Robot jojo= new Robot();
		jojo.avantMatch();
		jojo.marquerPremierPalet();
		//jojo.avancerDe(600);
		//while(jojo.chassis.isMoving()) {}
		//jojo.cherhcerObstaclePlusProche();
	}

}