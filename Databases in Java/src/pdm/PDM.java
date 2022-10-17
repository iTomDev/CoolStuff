/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package pdm;


/**
 *
 * @author Tom
 * Todo:
 * + fix remove record code
 * + add search
 */
public class PDM 
{

    /**
     * Testing the relational database
     * @param args the command line arguments
     */
    public static void main(String[] args) throws Exception
    {   
        // second round of testing
        // create a sample DB
        RelationalDatabase db = new RelationalDatabase();
        db.init();
        db.createDB();
        
        // add category
        int catid = db.createCategoryRecord("receipt");
        int shopid = db.createGUIComponentTitle(catid, "Shop", 1);
        int vatid = db.createGUIComponentTitle(catid, "VAT", 2);
        int priceid = db.createGUIComponentTitle(catid, "Price", 3);
        
        // add category
        db.createCategoryRecord("recipe");
        db.createGUIComponentTitle(catid, "Serving", 1);
        db.createGUIComponentTitle(catid, "Style", 2);
        db.createGUIComponentTitle(catid, "Rating", 3);
        
        // add category
        db.createCategoryRecord("shopping list");
        db.createGUIComponentTitle(catid, "week", 1);
        db.createGUIComponentTitle(catid, "Shop", 2);
        db.createGUIComponentTitle(catid, "", 3);
        
        // add a data record
        int recid = db.createDataRecord(catid, "speedboat");
        db.createGUIComponentValue(shopid, recid, "TescoValue", 0);
        db.createGUIComponentValue(vatid, recid, "2000", 0);
        db.createGUIComponentValue(priceid, recid, "10000", 0);
        // and another
        int recid2 = db.createDataRecord(catid, "large yacht");
        db.createGUIComponentValue(shopid, recid2, "Waitrose", 0);
        db.createGUIComponentValue(vatid, recid2, "20000", 0);
        db.createGUIComponentValue(priceid, recid2, "100000", 0);
        
        // access the data record
        String[][] recvals = db.getGUIComponentTriple(recid); 
        // loop through data to show its there
        
        for(int i=0; i<recvals.length;i++)
        {
            // loop cols
            for(int j=0; j<recvals[0].length;j++)
            {
                System.out.println(recvals[i][j]);
            }
        }
        
        // display a list of all the categories
        String catlist[][] = db.getCategoryList();
        for(int i=0; i<catlist.length;i++)
        {
            System.out.println(catlist[i][0] + " " + catlist[i][1]);
        }
        
        // display all of the labels for a given category id
        String[][] labellist = db.getComponentTitleList(catid);
        for(int i=0; i<labellist.length;i++)
        {
           // System.out.println(labellist[i][0] + " " + labellist[i][1]);
        }
        
        // display a list of the items in a given category
        String[][] reclist = db.getRecordListForCategory(catid);
        for(int i=0; i<reclist.length;i++)
        {
            System.out.println(reclist[i][0] + " " + reclist[i][1]);
        }
        
        /*
        // remove data record
        System.out.println("Highest UserData ID: "+db.getHighestUserDataID());
        db.removeDataRecord(recid);
        System.out.println("Highest UserData ID: "+db.getHighestUserDataID());
        
        // remove category record
        System.out.println("Highest category ID: "+db.getHighestCategoryID());
        db.removeCategoryRecord(catid);
        System.out.println("Highest category ID: "+db.getHighestCategoryID());
        */
        db.closeDBConnection();
    }
    
    public void loadGUI()
    {
        
    }
    
}
