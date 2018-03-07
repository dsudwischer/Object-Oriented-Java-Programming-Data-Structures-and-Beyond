package spelling;

import java.util.TreeSet;

/**
 * @author UC San Diego Intermediate MOOC team
 *
 */
public class DictionaryBST implements Dictionary 
{
   private TreeSet<String> dict;
   private boolean ignoreCase;
	  
 	// You'll need a constructor here
	public DictionaryBST(TreeSet<String> dict, boolean ignoreCase)
	{
		this.dict = dict;
		this.ignoreCase = ignoreCase;
	}
	
	public DictionaryBST(TreeSet<String> dict)
	{
		this(dict, true);
	}
	
	public DictionaryBST(boolean ignoreCase)
	{
		this(new TreeSet<String>(), ignoreCase);
	}
	
	public DictionaryBST()
	{
		this(new TreeSet<String>(), true);
	}
	
    /** Add this word to the dictionary.  Convert it to lowercase first
     * for the assignment requirements.
     * @param word The word to add
     * @return true if the word was added to the dictionary 
     * (it wasn't already there). */
	// I added a parameter to allow case sensitivity.
    public boolean addWord(String word)
    {
    	if(this.ignoreCase)
    	{
    		word = word.toLowerCase();
    	}
    	// If the dictionary contains the word, return false
        if(this.isWord(word))
        {
        	return false;
        }
        // else, add the word and return true
        this.dict.add(word);
        return true;
    }


    /** Return the number of words in the dictionary */
    public int size()
    {
    	return this.dict.size();
    }

    /** Is this a word according to this dictionary? */
    public boolean isWord(String s) {
    	if(this.ignoreCase)
    	{
    		s = s.toLowerCase();
    	}
        return this.dict.contains(s);
    }

}
