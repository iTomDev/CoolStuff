
package pdm;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.h2.tools.DeleteDbFiles;


/**
 *
 * @author Tom
 * Based on this:
 * https://www.javatips.net/blog/h2-database-example
 *
 * TODO:
 * - prevent duplicate records in GUItitle
 */
public class RelationalDatabase implements DataInterface
{
    Connection dbcon;
    private static final String DB_DRIVER = "org.h2.Driver";
    private static final String DB_CONNECTION = "jdbc:h2:~/test";
    private static final String DB_USER = "";
    private static final String DB_PASSWORD = "";

    /* Functions

    
    */
    
    public void init()
    {
        dbcon = getDBConnection();
        createDB();
        
        // add category
        int catid = createCategoryRecord("receipt");
        int shopid = createGUIComponentTitle(catid, "Shop", 1);
        int vatid = createGUIComponentTitle(catid, "VAT", 2);
        int priceid = createGUIComponentTitle(catid, "Price", 3);
        int imageid = createGUIComponentTitle(catid, "Image", 5);
        
        // add category
        int catidrpe = createCategoryRecord("recipe");
        createGUIComponentTitle(catidrpe, "Serving", 1);
        createGUIComponentTitle(catidrpe, "Style", 2);
        createGUIComponentTitle(catidrpe, "Rating", 3);
        
        // add category
        int catidsl = createCategoryRecord("shopping list");
        createGUIComponentTitle(catidsl, "week", 1);
        createGUIComponentTitle(catidsl, "Shop", 2);
        createGUIComponentTitle(catidsl, "", 3);
        createGUIComponentTitle(catidsl, "Image", 5);
        
        // add a data record for receipt
        //USERDATA(recordid int primary key, categoryid int, name varchar(128))
        //GUICOMPONENTVALUE( guicomponenttitleid int, recordid int, strvalue varchar(128), numvalue int)
        int recid = createDataRecord(catid, "speedboat");
        createGUIComponentValue(shopid, recid, "TescoValue", 0);
        createGUIComponentValue(vatid, recid, "2000", 0);
        createGUIComponentValue(priceid, recid, "10000", 0);
        createGUIComponentValue(imageid, recid, "speedboat.jpg", 0);
        
        // and another
        int recid2 = createDataRecord(catid, "large yacht");
        createGUIComponentValue(shopid, recid2, "Waitrose", 0);
        createGUIComponentValue(vatid, recid2, "20000", 0);
        createGUIComponentValue(priceid, recid2, "100000", 0);
        
        // other db testing
        String[][] temp = getComponentTitleList(1);
    }
    
    public void createDB()
    {
        //String SQLCreateTable_UserData = "CREATE TABLE IF NOT EXISTS UserData(recordid int primary key, foreign key (categoryid) references CATEGORY(categoryid), foreign key (boxvalueid) references BOXVALUE, varchar(128) title, dateval date) ";
        String SQLCreateSchema = "CREATE SCHEMA IF NOT EXISTS PDM";
        String SQLCreateTable_UserData = "CREATE TABLE IF NOT EXISTS PDM.USERDATA(recordid int primary key, categoryid int, name varchar(128));";
        String SQLCreateCategoryTable = "CREATE TABLE IF NOT EXISTS PDM.CATEGORY(categoryid int primary key, name varchar(128));";
        String SQLCreateTable_GUIComponentTitle = "CREATE TABLE IF NOT EXISTS PDM.GUICOMPONENTTITLE(guicomponenttitleid int primary key, categoryid int, label varchar(128), boxnumber int)";
        String SQLCreateTable_GUIComponentValue = "CREATE TABLE IF NOT EXISTS PDM.GUICOMPONENTVALUE(guicomponentvalueid int primary key, guicomponenttitleid int, recordid int, strvalue varchar(128), numvalue int)";
        String SQLLinkTables1 = "ALTER TABLE PDM.USERDATA ADD FOREIGN KEY (categoryid) REFERENCES PDM.CATEGORY(categoryid);";
        String SQLLinkTables2 = "ALTER TABLE PDM.GUIComponentTitle ADD FOREIGN KEY (categoryid) REFERENCES PDM.CATEGORY(categoryid);";
        String SQLLinkTables3 = "ALTER TABLE PDM.GUIComponentValue ADD FOREIGN KEY (guicomponenttitleid) REFERENCES PDM.GUIComponentTitle(guicomponenttitleid);";
        String SQLLinkTables4 = "ALTER TABLE PDM.GUIComponentValue ADD FOREIGN KEY (recordid) REFERENCES PDM.USERDATA(recordid);";
        //
        try
        {
            dbcon.setAutoCommit(false);
            // Create schema. only run if you've deleted the old db!
            PreparedStatement createSchemaPS = dbcon.prepareStatement(SQLCreateSchema);
            createSchemaPS.executeUpdate();
            createSchemaPS.close();
            // Create Category table
            PreparedStatement createCategoryPS = dbcon.prepareStatement(SQLCreateCategoryTable);
            createCategoryPS.executeUpdate();
            createCategoryPS.close();
            // Create UserData table
            PreparedStatement createUserdataPS = dbcon.prepareStatement(SQLCreateTable_UserData);
            createUserdataPS.executeUpdate();
            createUserdataPS.close();
            // Create GUIComponentTitle table
            PreparedStatement createGUIComponentTitlePS = dbcon.prepareStatement(SQLCreateTable_GUIComponentTitle);
            createGUIComponentTitlePS.executeUpdate();
            createGUIComponentTitlePS.close();
            // Create GUIComponentTitle table
            PreparedStatement createGUIComponentValuePS = dbcon.prepareStatement(SQLCreateTable_GUIComponentValue);
            createGUIComponentValuePS.executeUpdate();
            createGUIComponentValuePS.close();
            // Link tables 1
            PreparedStatement createTableLink1PS = dbcon.prepareStatement(SQLLinkTables1);
            createTableLink1PS.executeUpdate();
            createTableLink1PS.close();
            // Link tables 2
            PreparedStatement createTableLink2PS = dbcon.prepareStatement(SQLLinkTables2);
            createTableLink2PS.executeUpdate();
            createTableLink2PS.close();
            // Link tables 3
            PreparedStatement createTableLink3PS = dbcon.prepareStatement(SQLLinkTables3);
            createTableLink3PS.executeUpdate();
            createTableLink3PS.close();
            // Link tables 4
            PreparedStatement createTableLink4PS = dbcon.prepareStatement(SQLLinkTables4);
            createTableLink4PS.executeUpdate();
            createTableLink4PS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    /* 
    Takes the name of the category, auto generates the category id
    returns the category id
    */
    public int createCategoryRecord(String name)
    {
        String SQLAddCategory = "INSERT INTO PDM.CATEGORY" + "(categoryid, name) values" + "(?,?)";
        int newcatid = 0;
        
        try
        {
            // 
            newcatid = getHighestCategoryID() +1;
            
            // add a category record
            PreparedStatement addCategoryPS;
            addCategoryPS = dbcon.prepareStatement(SQLAddCategory);
            addCategoryPS.setInt(1,newcatid);
            addCategoryPS.setString(2, name);
            addCategoryPS.executeUpdate();
            addCategoryPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        return newcatid;
    }
    
    public int createDataRecord(int categoryid, String name)
    {
        String SQLAddUserdata = "INSERT INTO PDM.USERDATA" + "(recordid, categoryid, name) values" + "(?,?,?)";
        int newrecid = 0;
        
        try
        {
            // 
            newrecid = getHighestUserDataID() +1;
            
            // add UserData record
            PreparedStatement addUserDataPS;
            addUserDataPS = dbcon.prepareStatement(SQLAddUserdata);
            addUserDataPS.setInt(1, newrecid); // recordid
            addUserDataPS.setInt(2, categoryid); // categoryid
            addUserDataPS.setString(3, name);
            addUserDataPS.executeUpdate();
            addUserDataPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        return newrecid;
    }
    
    public int createGUIComponentValue(int guicomponenttitleid, int recordid, String strvalue, int numvalue)
    {
        String SQLAddUserdata = "INSERT INTO PDM.GUICOMPONENTVALUE" + "(guicomponentvalueid, guicomponenttitleid, recordid, strvalue, numvalue) values" + "(?,?,?,?,?)";
        int newrecid = 0;
        
        try
        {
            // 
            newrecid = getHighestComponentValueID() +1;
            
            // add UserData record
            PreparedStatement addUserDataPS;
            addUserDataPS = dbcon.prepareStatement(SQLAddUserdata);
            addUserDataPS.setInt(1, newrecid); // guicomponentvalueid
            addUserDataPS.setInt(2, guicomponenttitleid); // guicomponenttitleid
            addUserDataPS.setInt(3, recordid); // recordid
            addUserDataPS.setString(4, strvalue);
            addUserDataPS.setInt(5, numvalue); 
            addUserDataPS.executeUpdate();
            addUserDataPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        return newrecid;
    }    
    
    public int createGUIComponentTitle(int categoryid, String label, int boxnumber)
    {
        String SQLAddUserdata = "INSERT INTO PDM.GUICOMPONENTTITLE" + "(guicomponenttitleid, categoryid, label, boxnumber) values" + "(?,?,?,?)";
        int newrecid = 0;
        
        try
        {
            // 
            newrecid = getHighestComponentTitleID() +1;
            
            // add UserData record
            PreparedStatement addGUIComponentTitlePS;
            addGUIComponentTitlePS = dbcon.prepareStatement(SQLAddUserdata);
            addGUIComponentTitlePS.setInt(1, newrecid); // guicomponenttitleid
            addGUIComponentTitlePS.setInt(2, categoryid); 
            addGUIComponentTitlePS.setString(3, label); // gui label
            addGUIComponentTitlePS.setInt(4, boxnumber); 
            addGUIComponentTitlePS.executeUpdate();
            addGUIComponentTitlePS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        return newrecid;
    } 
    
    public int getHighestCategoryID()
    {
        int ret = 0;
        String SQLGetMaxCategoryID = "SELECT MAX(categoryid) AS maxid from PDM.CATEGORY";
        try
        {
            // get the higheest category ID
            //dbcon.setAutoCommit(false);
            PreparedStatement getMaxCategoryidPS = dbcon.prepareStatement(SQLGetMaxCategoryID);
            ResultSet rs;
            rs = getMaxCategoryidPS.executeQuery();
            while (rs.next()==true)
            {
                ret = rs.getInt("maxid");
                //System.out.println(ret);
            }
            getMaxCategoryidPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        return ret;
    }
    
    public int getHighestUserDataID()
    {
        int ret = 0;
        String SQLGetMaxCategoryID = "SELECT MAX(recordid) AS maxid from PDM.USERDATA";
        try
        {
            // get the higheest category ID
            dbcon.setAutoCommit(false);
            PreparedStatement getMaxCategoryidPS = dbcon.prepareStatement(SQLGetMaxCategoryID);
            ResultSet rs;
            rs = getMaxCategoryidPS.executeQuery();
            while (rs.next()==true)
            {
                ret = rs.getInt("maxid");
                //System.out.println(ret);
            }
            getMaxCategoryidPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        return ret;
    }
    
    public int getHighestComponentValueID()
    {
        int ret = 0;
        String SQLGetMaxCategoryID = "SELECT MAX(guicomponentvalueid) AS maxid from PDM.GUICOMPONENTVALUE";
        try
        {
            // get the higheest category ID
            dbcon.setAutoCommit(false);
            PreparedStatement getMaxCategoryidPS = dbcon.prepareStatement(SQLGetMaxCategoryID);
            ResultSet rs;
            rs = getMaxCategoryidPS.executeQuery();
            while (rs.next()==true)
            {
                ret = rs.getInt("maxid");
                //System.out.println(ret);
            }
            getMaxCategoryidPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        return ret;
    }
    
    public int getHighestComponentTitleID()
    {
        int ret = 0;
        String SQLGetMaxCategoryID = "SELECT MAX(guicomponenttitleid) AS maxid from PDM.GUICOMPONENTTITLE";
        try
        {
            // get the higheest category ID
            dbcon.setAutoCommit(false);
            PreparedStatement getMaxCategoryidPS = dbcon.prepareStatement(SQLGetMaxCategoryID);
            ResultSet rs;
            rs = getMaxCategoryidPS.executeQuery();
            while (rs.next()==true)
            {
                ret = rs.getInt("maxid");
                //System.out.println(ret);
            }
            getMaxCategoryidPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        return ret;
    }
    
    /*
    Removes a record from UserData and all of the associated GUI data values too
    */
    public void removeDataRecord(int recordid)
    {
        String SQLRemoveUserData = "DELETE FROM PDM.USERDATA WHERE recordid=?";
        String SQLRemoveGUIComponentValue = "DELETE FROM PDM.GUICOMPONENTVALUE WHERE recordid=?";
        try
        {
            // remove all gui value records for given userdata recordid
            
            // remove gui components
            dbcon.setAutoCommit(false);
            PreparedStatement RemoveGUIComponentValuePS = dbcon.prepareStatement(SQLRemoveGUIComponentValue);
            RemoveGUIComponentValuePS.setInt(1, recordid);
            RemoveGUIComponentValuePS.execute();
            RemoveGUIComponentValuePS.close();
            
            // remove user record
            dbcon.setAutoCommit(false);
            PreparedStatement RemoveUserDataPS = dbcon.prepareStatement(SQLRemoveUserData);
            RemoveUserDataPS.setInt(1, recordid);
            RemoveUserDataPS.execute();
            RemoveUserDataPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    /*
    Removes a record from Category and all of the associated GUI data titles too
    */
    public void removeCategoryRecord(int categoryid)
    {
        String SQLRemoveCategory = "DELETE FROM PDM.CATEGORY WHERE categoryid=?";
        String SQLRemoveGUIComponentValue = "DELETE FROM PDM.GUICOMPONENTVALUE WHERE recordid=?";
        String SQLRemoveGUIComponentTitle = "DELETE FROM PDM.GUICOMPONENTTITLE WHERE categoryid=?";
        try
        {
            // find any records with that category id, or else the component will be undefined
            String[][] reclist = getRecordListForCategory(categoryid);
            // extract the ids as ints
            for(int i=0;i<reclist.length;i++)
            {
                int recid = Integer.parseInt(reclist[i][0]);
                
                // remove gui component values
                dbcon.setAutoCommit(false);
                PreparedStatement RemoveGUIComponentValuePS = dbcon.prepareStatement(SQLRemoveGUIComponentValue);
                RemoveGUIComponentValuePS.setInt(1, recid);
                RemoveGUIComponentValuePS.execute();
                RemoveGUIComponentValuePS.close();
                        
                // call remove data records to remove them
                removeDataRecord(recid);
            }
            
            // remove gui component titles
            dbcon.setAutoCommit(false);
            PreparedStatement RemoveGUIComponentTitlePS = dbcon.prepareStatement(SQLRemoveGUIComponentTitle);
            RemoveGUIComponentTitlePS.setInt(1, categoryid);
            RemoveGUIComponentTitlePS.execute();
            RemoveGUIComponentTitlePS.close();
            
            // remove user record
            dbcon.setAutoCommit(false);
            PreparedStatement RemoveCategoryPS = dbcon.prepareStatement(SQLRemoveCategory);
            RemoveCategoryPS.setInt(1, categoryid);
            RemoveCategoryPS.execute();
            RemoveCategoryPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    /*
     returns nx2, cols: categoryid, name
    */
    public String[][] getCategoryList()
    {
        String SQLGetCategoryList = "SELECT * FROM PDM.CATEGORY;";
        // temp return array
        String[][] results = new String[500][2];
        int iresults = 0;
        
        try
        {
            // get a list of all categories
            dbcon.setAutoCommit(false);
            PreparedStatement getCategoryListPS = dbcon.prepareStatement(SQLGetCategoryList);
            ResultSet rs = null; 
            rs = getCategoryListPS.executeQuery();
            
            while (rs.next()==true)
            {
                //System.out.println("CategoryID: "+rs.getInt("categoryid"));
                //System.out.println("Category Name: "+rs.getString("name"));

                // put in an array and return
                results[iresults][0] = String.valueOf(rs.getInt("categoryid"));
                results[iresults][1] = rs.getString("name");
                iresults++;
            }
            getCategoryListPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        // trim array
        String[][] ret = new String[iresults][2];
        for(int i=0;i<iresults;i++)
        {
            ret[i][0] = results[i][0];
            ret[i][1] = results[i][1];
        }
        return ret;
    }
    
    /*
    Used to list the items under a given category
    includes recordid so the record data can be looked up more easily
    returns nx2, cols: recordid, name
    */
    public String[][] getRecordListForCategory(int categoryid)
    {
        String SQLGetRecords = "SELECT * FROM PDM.USERDATA WHERE categoryid=?";
        // temp return array
        String[][] results = new String[500][2];
        int iresults = 0;
        try
        {
            // get a list of all categories
            dbcon.setAutoCommit(false);
            PreparedStatement getRecordsPS = dbcon.prepareStatement(SQLGetRecords);
            getRecordsPS.setInt(1,categoryid);
            ResultSet rs = null; 
            rs = getRecordsPS .executeQuery();
            while (rs.next()==true)
            {
                //System.out.println("RecordID: "+rs.getInt("recordid"));
                //System.out.println("Item Name: "+rs.getString("name"));

                // put in an array and return
                results[iresults][0] = String.valueOf(rs.getInt("recordid"));
                results[iresults][1] = rs.getString("name");
                iresults++;
            }
            getRecordsPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        // trim array
        String[][] ret = new String[iresults][2];
        for(int i=0;i<iresults;i++)
        {
            ret[i][0] = results[i][0];
            ret[i][1] = results[i][1];
        }
        return ret;
    }
    
    /*
    Gets the component titles for a given category
    Used to fill out the labels in an empty form
    includes associated id to make it easier when making new data records
    
     returns nx3, cols: componenttitleid, name, boxnumber
    */
    public String[][] getComponentTitleList(int categoryid)
    {
        String SQLGetComponentTitleList = "SELECT * FROM PDM.GUICOMPONENTTITLE where categoryid=?;";
        // temp return array
        String[][] results = new String[500][3];
        int iresults = 0;
            
        try
        {
            dbcon.setAutoCommit(false);
            PreparedStatement getComponentTitletPS = dbcon.prepareStatement(SQLGetComponentTitleList);
            getComponentTitletPS.setInt(1,categoryid);
            ResultSet rs = null; 
            rs = getComponentTitletPS.executeQuery();
            while (rs.next()==true)
            {
                //System.out.println("CategoryID: "+rs.getInt("categoryid"));
                //System.out.println("Category Name: "+rs.getString("name"));

                // put in an array and return
                results[iresults][0] = String.valueOf(rs.getInt("guicomponenttitleid"));
                results[iresults][1] = rs.getString("label");
                results[iresults][2] = String.valueOf(rs.getInt("boxnumber"));
                iresults++;
            }
            getComponentTitletPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        // trim array
        String[][] ret = new String[iresults][3];
        for(int i=0;i<iresults;i++)
        {
            ret[i][0] = results[i][0];
            ret[i][1] = results[i][1];
            ret[i][2] = results[i][2];
        }
        return ret;
    }
    
    /*
    Returns all GUI component values and their labels for a given data record e.g 
    for a receipt
    returns nx4, cols: label, stringvalue, numbervalue, boxnumber
    */
    public String[][] getGUIComponentTriple(int recordid)
    {
        String SQLGetGUIValues = "SELECT * FROM PDM.GUICOMPONENTVALUE WHERE recordid=?";
        String SQLGetGUITitles = "SELECT * FROM PDM.GUICOMPONENTTITLE WHERE guicomponenttitleid=?";
        // temp return array
        String[][] results = new String[500][4];
        int iresults = 0;
            
        try
        {
            int[] guicomponenttitleidarr = new int[50];
            
            // get the component values
            dbcon.setAutoCommit(false);
            PreparedStatement getGUIValuesPS = dbcon.prepareStatement(SQLGetGUIValues);
            getGUIValuesPS.setInt(1, recordid);
            ResultSet rs = null; 
            rs = getGUIValuesPS.executeQuery();
            while (rs.next()==true)
            {
                //System.out.println("guicomponenttitleid: "+rs.getInt("guicomponenttitleid"));
                //System.out.println("strvalue: "+rs.getString("strvalue"));
                //System.out.println("numvalue: "+rs.getInt("numvalue"));

                // put in an array and return
                //results[iresults][0] = String.valueOf(rs.getInt("guicomponenttitleid"));
                guicomponenttitleidarr[iresults] = rs.getInt("guicomponenttitleid");
                results[iresults][1] = rs.getString("strvalue");
                results[iresults][2] = String.valueOf(rs.getInt("numvalue"));
                iresults++;
                
            }
            getGUIValuesPS.close();
            
            // get the gui titles for the category defined in the record
            //if(categoryid>0 && categoryid!=null)
            for(int i=0; i<iresults; i++)
            {
                // get the gui component titles 
                dbcon.setAutoCommit(false);
                PreparedStatement getGUITitlesPS = dbcon.prepareStatement(SQLGetGUITitles);
                getGUITitlesPS.setInt(1, guicomponenttitleidarr[i]); // get the title for a given guicomponenttitleid
                rs = getGUITitlesPS.executeQuery();
                while (rs.next()==true)
                {
                    //System.out.println("label: "+rs.getString("label"));
                    
                    // put the label with its data values counterparts
                    results[i][0] = rs.getString("label");
                    results[i][3] = String.valueOf(rs.getInt("boxnumber"));
                }
                getGUIValuesPS.close();
            }
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        // trim array
        String[][] ret = new String[iresults][4];
        for(int i=0;i<iresults;i++)
        {
            ret[i][0] = results[i][0];
            ret[i][1] = results[i][1];
            ret[i][2] = results[i][2];
            ret[i][3] = results[i][3];
            //System.out.println(results[i][0]+" "+results[i][1]+" "+results[i][2]);
            
        }
        return ret;
    }
    
    public String getUserDataName(int recordid)
    {
        String SQLGetGUIValues = "SELECT * FROM PDM.USERDATA WHERE recordid=?";
        // temp return array
        String results = "";
            
        try
        {
            // get the component values
            dbcon.setAutoCommit(false);
            PreparedStatement getGUIValuesPS = dbcon.prepareStatement(SQLGetGUIValues);
            getGUIValuesPS.setInt(1, recordid);
            ResultSet rs = null; 
            rs = getGUIValuesPS.executeQuery();
            while (rs.next()==true)
            {
                results = rs.getString("name");
            }
            getGUIValuesPS.close();
        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
        return results;
    }
    
    public void getCategoryRecord()
    {
        String SQLGetCategory = "SELECT * FROM PDM.CATEGORY;";
        String SQLGetCategory1 = "SELECT * FROM PDM.CATEGORY WHERE ;";
        String SQLGetMaxCategoryID = "SELECT MAX(categoryid) AS maxid from PDM.CATEGORY";
        String SQLGetCategoryList = "SELECT * FROM PDM.CATEGORY;";
        try
        {

            // get all records from category
            dbcon.setAutoCommit(false);
            PreparedStatement getCategoryPS = dbcon.prepareStatement(SQLGetCategory);
            ResultSet rs = getCategoryPS.executeQuery();
            while (rs.next()==true)
            {
                System.out.println("Id "+rs.getString("name"));
            }
            getCategoryPS.close();

            // get the higheest category ID
            dbcon.setAutoCommit(false);
            PreparedStatement getMaxCategoryidPS = dbcon.prepareStatement(SQLGetMaxCategoryID);
            rs = getMaxCategoryidPS.executeQuery();
            while (rs.next()==true)
            {
                System.out.println("Id "+rs.getInt("maxid"));
            }
            getMaxCategoryidPS.close();

            // get a list of all categories
            dbcon.setAutoCommit(false);
            PreparedStatement getCategoryListPS = dbcon.prepareStatement(SQLGetCategoryList);
            rs = getCategoryListPS.executeQuery();
            while (rs.next()==true)
            {
                //System.out.println("CategoryID: "+rs.getInt("categoryid"));
                //System.out.println("Category Name: "+rs.getString("name"));

                // put in an array and return
            }
            getCategoryListPS.close();

            // get all records from db with categoryid

            // get user record data and from linked records

        }
        catch (SQLException ex)
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    
    
    private static Connection getDBConnection() 
    {
        Connection dbConnection = null;
        try {
            Class.forName(DB_DRIVER);
        } catch (ClassNotFoundException e) {
            System.out.println(e.getMessage());
        }
        try {
            dbConnection = DriverManager.getConnection(DB_CONNECTION, DB_USER,
                    DB_PASSWORD);
            return dbConnection;
        } catch (SQLException e) {
            System.out.println(e.getMessage());
        }
        return dbConnection;
    }
    
    public void closeDBConnection()
    {
        //
        try 
        {
            dbcon.close();
        } catch (SQLException ex) 
        {
            Logger.getLogger(RelationalDatabase.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

}
