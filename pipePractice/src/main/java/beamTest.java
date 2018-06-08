import org.apache.beam.sdk.Pipeline;
import org.apache.beam.sdk.io.TextIO;
import org.apache.beam.sdk.options.PipelineOptions;
import org.apache.beam.sdk.options.PipelineOptionsFactory;
import org.apache.beam.sdk.transforms.*;
import org.apache.beam.sdk.values.KV;
import org.apache.beam.sdk.values.PCollection;
import org.apache.beam.sdk.values.TypeDescriptors;

import javax.xml.soap.Text;
import java.lang.reflect.Type;
import java.util.Arrays;


public class beamTest
{

    public static class ExtractWordsFn extends DoFn<String, String>
    {
        @ProcessElement
        public void processElement(ProcessContext data)
        {
            String word = data.element();
            String[] strings = word.split("[^\\p{L}]+");
            for (String string : strings) {
                data.output(string);
            }
        }
    }


    public static class FormatAsTextFn extends SimpleFunction<KV<String, Long>, String>
    {
        @Override
        public String apply(KV<String, Long> input)
        {
            return input.getKey() + ": " + input.getValue();
        }

    }

    public static class CountWords extends PTransform<PCollection<String>, PCollection<KV<String, Long>>>
    {

        public PCollection<KV<String, Long>> expand(PCollection<String> data)
        {
            PCollection<String> words = data.apply(ParDo.of(new ExtractWordsFn()));
            PCollection<KV<String, Long>> wordCounts = words.apply(Count.perElement());

            return wordCounts;
        }
    }

    static void runWordCount(PipelineOptions op)
    {
        Pipeline p = Pipeline.create(op);
        p.apply("ReadLines", TextIO.read().from("C:\\Users\\moslea\\Dropbox\\EssexLG\\pipePractice\\test.txt"))
                .apply(new CountWords())
                .apply(MapElements.via(new FormatAsTextFn()))
                .apply("WriteCounts", TextIO.write().to("WC"));
        p.run().waitUntilFinish();
    }




    public static void main(String[] args)
    {
        PipelineOptions options = PipelineOptionsFactory.fromArgs(args).withValidation().create();

        runWordCount(options);
    }

}
