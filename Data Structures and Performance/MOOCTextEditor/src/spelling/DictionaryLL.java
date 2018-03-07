package spelling;

import java.util.LinkedList;

/**
 * A class that implements the Dictionary interface using a LinkedList
 *
 */
public class DictionaryLL implements Dictionary 
{

	private LinkedList<String> dict;
	private boolean ignoreCase;
	
    public DictionaryLL(LinkedList<String> dict, boolean ignoreCase)
    {
    	this.dict = dict;
    	this.ignoreCase = ignoreCase;
    }
    
    public DictionaryLL(LinkedList<String> dict)
    {
    	this(dict, true);
    }
    
    public DictionaryLL(boolean ignoreCase)
    {
    	this(new LinkedList<String>(), ignoreCase);
    }
    
    public DictionaryLL()
    {
    	this(new LinkedList<String>(), true);
    }


    /** Add this word to the dictionary.  Convert it to lowercase first
     * for the assignment requirements.
     * @param word The word to add
     * @return true if the word was added to the dictionary 
     * (it wasn't already there). */
    
    // I will add an optional parameter to account for case
    public boolean addWord(String word)
    {
    	if(this.ignoreCase)
    	{
    		word = word.toLowerCase();
    	}
    	// If the word is already in the dictionary, return false
    	if(isWord(word))
    	{
    		return false;
    	}
    	// else, add it to the dictionary and return true.
        this.dict.add(word);
        return true;
    }


    /** Return the number of words in the dictionary */
    public int size()
    {
        return this.dict.size();
    }

    /** Is this a word according to this dictionary? */
    public boolean isWord(String s)
    {
    	if(this.ignoreCase)
    	{
    		// Convert to lower case first
    		s = s.toLowerCase();
    	}
    	// If the word is contained in the dictionary, return true
        return this.dict.contains(s);
    }

    
}
