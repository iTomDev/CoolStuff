/*
 */
package pdm;

/**
 *
 * @author Tom
 */
public interface DataInterface 
{
    final int CATEGORY_LIST_MAXIMUM = 100;
    final int ITEM_LIST_MAXIMUM = 100;
    
    /*
    public void readDataFile();
    public String[] getCurrentUserRecordByIndex(int x);
    public void removeUserDataRecordByIndex(int currIndexUserData);
    public String[][] getItemsList(String category);
    public int getUserDataRecordByTitle(String title);
    public void setRecordAttribute(int currIndexUserData, int column, String val);
    public int searchRecords(String term);
    public int saveDataFile();
    public String[] getCategoryList();
    */
    public void init();
    public void createDB();
    public int createCategoryRecord(String name);
    public int createDataRecord(int categoryid, String name);
    public int createGUIComponentValue(int guicomponenttitleid, int recordid, String strvalue, int numvalue);
    public int createGUIComponentTitle(int categoryid, String label, int boxnumber);
    public int getHighestCategoryID();
    public int getHighestUserDataID();
    public int getHighestComponentValueID();
    public int getHighestComponentTitleID();
    public void removeDataRecord(int recordid);
    public void removeCategoryRecord(int categoryid);
    public String[][] getCategoryList();
    public String[][] getRecordListForCategory(int categoryid);
    public String[][] getComponentTitleList(int categoryid);
    public String[][] getGUIComponentTriple(int recordid);
    public void getCategoryRecord();
    public String getUserDataName(int recordid);
    public void closeDBConnection();
}
