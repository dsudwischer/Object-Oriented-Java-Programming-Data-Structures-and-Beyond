package textgen;

import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Random;

/** 
 * An implementation of the MTG interface that uses a list of lists.
 * @author UC San Diego Intermediate Programming MOOC team 
 */
public class MarkovTextGeneratorLoL implements MarkovTextGenerator {

	// The list of words with their next words
	private List<ListNode> wordList; 
	
	// The starting "word"
	private String starter;
	
	// The random number generator
	private Random rnGenerator;
	
	public MarkovTextGeneratorLoL(Random generator)
	{
		wordList = new LinkedList<ListNode>();
		starter = "";
		rnGenerator = generator;
	}
	
	
	/** Train the generator by adding the sourceText */
	@Override
	public void train(String sourceText)
	{
		// Since this assignment explicitly asks for an implementation using lists of lists,
		// the performance will be quite bad (quadratic worst case runtime for training).
		// Additionally, we will use a dense representation which is highly unsuitable for long texts.
		// Punctuation and lower and upper case will be distinguished.
		String[] words = sourceText.split("[\\s]+");
		int numWords = words.length;
		if (numWords <= 0 || sourceText.equals(""))
		{
			return;
		}
		starter = words[0];
		String prevWord = words[numWords - 1]; // We start with the last word in the text as previous word
		for (String word : words)
		{
			// word is the current word
			ListNode node = findNode(prevWord);
			// If the previous word is already contained, we just add the current word
			if (node != null)
			{
				node.addNextWord(word);
			}
			else // or else, we will construct the node and add it to the generator
			{
				node = new ListNode(prevWord);
				node.addNextWord(word);
				wordList.add(node);
			}
			prevWord = word;
		}
		// To be in line with the order in the example...
		wordList.add(wordList.get(0));
		wordList.remove(0);
	}
	
	// Returns the ListNode object that contains the specified string as word.
	private ListNode findNode(String toFind)
	{
		for (ListNode node : wordList)
		{
			if (node.getWord().equals(toFind))
			{
				return node;
			}
		}
		return null;
	}
	
	
	/** 
	 * Generate the number of words requested.
	 */
	@Override
	public String generateText(int numWords)
	{
	    String output = new String();
	    String currentWord = starter;
	    if (starter.equals(""))
	    {
	    	return output;
	    }
	    for (int i = 0; i < numWords; i++)
	    {
	    	output = output.concat(currentWord.concat(" "));
	    	ListNode nextListNode = findNode(currentWord);
	    	currentWord = nextListNode.getRandomNextWord(rnGenerator);
	    }
		return output;
	}
	
	
	// Can be helpful for debugging
	@Override
	public String toString()
	{
		String toReturn = "";
		for (ListNode n : wordList)
		{
			toReturn += n.toString();
		}
		return toReturn;
	}
	
	/** Retrain the generator from scratch on the source text */
	@Override
	public void retrain(String sourceText)
	{
		starter = "";
		wordList = new LinkedList<ListNode>();
		train(sourceText);
	}
	
	
	/**
	 * This is a minimal set of tests.  Note that it can be difficult
	 * to test methods/classes with randomized behavior.   
	 * @param args
	 */
	public static void main(String[] args)
	{
		// feed the generator a fixed random value for repeatable behavior
		MarkovTextGeneratorLoL gen = new MarkovTextGeneratorLoL(new Random(42));
		String textString = "Hello.  Hello there.  This is a test.  Hello there.  Hello Bob.  Test again.";
		System.out.println(textString);
		gen.train(textString);
		System.out.println(gen);
		System.out.println(gen.generateText(20));
		String textString2 = "You say yes, I say no, "+
				"You say stop, and I say go, go, go, "+
				"Oh no. You say goodbye and I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello. "+
				"I say high, you say low, "+
				"You say why, and I say I don't know. "+
				"Oh no. "+
				"You say goodbye and I say hello, hello, hello. "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello. "+
				"Why, why, why, why, why, why, "+
				"Do you say goodbye. "+
				"Oh no. "+
				"You say goodbye and I say hello, hello, hello. "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello. "+
				"You say yes, I say no, "+
				"You say stop and I say go, go, go. "+
				"Oh, oh no. "+
				"You say goodbye and I say hello, hello, hello. "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello, hello, hello,";
		System.out.println(textString2);
		gen.retrain(textString2);
		System.out.println(gen);
		System.out.println(gen.generateText(20));
	}

}

/** Links a word to the next words in the list 
 * You should use this class in your implementation. */
class ListNode
{
    // The word that is linking to the next words
	private String word;
	
	// The next words that could follow it
	private List<String> nextWords;
	
	ListNode(String word)
	{
		this.word = word;
		nextWords = new LinkedList<String>();
	}
	
	public String getWord()
	{
		return word;
	}

	public void addNextWord(String nextWord)
	{
		nextWords.add(nextWord);
	}
	
	public String getRandomNextWord(Random generator)
	{
		int numNextWords = nextWords.size();
	    int nextIndex = generator.nextInt(numNextWords);
	    return nextWords.get(nextIndex);
	}

	public String toString()
	{
		String toReturn = word + ": ";
		for (String s : nextWords) {
			toReturn += s + "->";
		}
		toReturn += "\n";
		return toReturn;
	}
	
}


